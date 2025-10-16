from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
from libcamera import controls

import threading
import sys
import time
from datetime import datetime
import cv2 as cv
import numpy as np
import glob
import os
import random

import config

import cv2
import re
from pathlib import Path


# =========================
# Undistorted-domain helper
# =========================

class Undistorter:
    """
    Build once per (K, dist, raw_size[, new_size, alpha]).
    - undistort(): returns an undistorted BGR image of size new_size
    - K_new: intrinsics in undistorted domain
    - dist_zero: zeros for undistorted geometry
    """
    def __init__(self, K, dist, raw_size, alpha=0.0, new_size=None):
        self.K = K.astype(np.float32)
        self.dist = dist.astype(np.float32)
        self.raw_size = (int(raw_size[0]), int(raw_size[1]))  # (width, height)
        self.new_size = (int(new_size[0]), int(new_size[1])) if new_size else self.raw_size

        self.K_new, self.roi = cv.getOptimalNewCameraMatrix(
            self.K, self.dist, self.raw_size, alpha, self.new_size
        )
        self.map1, self.map2 = cv.initUndistortRectifyMap(
            self.K, self.dist, np.eye(3, dtype=np.float32), self.K_new,
            self.new_size, cv.CV_32FC1
        )
        self.dist_zero = np.zeros_like(self.dist, dtype=np.float32)

    def undistort(self, frame_bgr):
        return cv.remap(frame_bgr, self.map1, self.map2, interpolation=cv.INTER_LINEAR)


# =========================
# ArUco pipeline (both modes)
# =========================

class ArucoPipeline:
    """
    One pipeline to process frames:
    - If config.UNDISTORT is True:
        undistort → detect → solvePnP (K_new, dist=0) → project → draw → (optional) save
    - If config.UNDISTORT is False:
        detect → solvePnP (K, dist) → project → draw → (optional) save

    Intrinsics and crops/resizes are kept consistent in either mode.
    """
    def __init__(self, K, dist, alpha=0.0, new_size=None):
        self.K = K
        self.dist = dist
        self.alpha = alpha
        self.new_size = new_size

        self.undistorter = None  # created lazily from first frame size (only if UNDISTORT=True)
        self.prev_rvec = None
        self.prev_tvec = None

        # ArUco detector
        dictionary = cv.aruco.getPredefinedDictionary(config.ARUCO_DICT)
        parameters = cv.aruco.DetectorParameters()
        if getattr(config, "ADVANCED_PARAMETERS", False):
            p = parameters
            # p.adaptiveThreshWinSizeMin = 3    # slow down too much
            # p.adaptiveThreshWinSizeMax = 53   # slow down too much
            # p.adaptiveThreshWinSizeStep = 4   # slow down too much
            p.adaptiveThreshConstant = 7
            p.minMarkerPerimeterRate = 0.01
            p.maxMarkerPerimeterRate = 6.0
            p.polygonalApproxAccuracyRate = 0.03
            p.markerBorderBits = 1
            p.maxErroneousBitsInBorderRate = 0.5
            p.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
            p.cornerRefinementWinSize = 7
            p.cornerRefinementMaxIterations = 100
            p.cornerRefinementMinAccuracy = 0.005
            p.perspectiveRemovePixelPerCell = 10
            p.perspectiveRemoveIgnoredMarginPerCell = 0.33
            p.minCornerDistanceRate = 0.03
            p.minMarkerDistanceRate = 0.02
            p.detectInvertedMarker = True

        self.detector = cv.aruco.ArucoDetector(dictionary, parameters)

    # ---------- shared helpers ----------

    @staticmethod
    def _apply_user_ops(gray, K_work):
        """
        Apply optional user crop (%), RECENTER_ORIGIN, PRE_PROCESS (resize/CLAHE/sharpen),
        updating intrinsics accordingly. Works for either domain.
        """
        # User crop (% of width/height)
        if getattr(config, "CROP_X", 0) > 0 or getattr(config, "CROP_Y", 0) > 0:
            H, W = gray.shape[:2]
            x1 = int(W * config.CROP_X / 2)
            y1 = int(H * config.CROP_Y / 2)
            x2 = int(W * (1 - config.CROP_X / 2))
            y2 = int(H * (1 - config.CROP_Y / 2))
            gray = gray[y1:y2, x1:x2]
            K_work[0, 2] -= x1
            K_work[1, 2] -= y1

        # Recenter principal point (optional)
        if getattr(config, "RECENTER_ORIGIN", False):
            H, W = gray.shape[:2]
            K_work[0, 2] = W / 2.0
            K_work[1, 2] = H / 2.0

        # Pre-process (resize + local contrast + sharpen)
        if getattr(config, "PRE_PROCESS", False):
            sx = float(config.RES_DROP_PRE)
            sy = float(config.RES_DROP_PRE)
            gray = cv.resize(gray, (0, 0), fx=sx, fy=sy, interpolation=cv.INTER_AREA)

            K_work[0, 0] *= sx
            K_work[1, 1] *= sy
            K_work[0, 2] *= sx
            K_work[1, 2] *= sy

            clahe = cv.createCLAHE(clipLimit=1.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)

        return gray, K_work

    def _ensure_undistorter(self, frame_bgr):
        if not getattr(config, "UNDISTORT", True):
            return
        if self.undistorter is None:
            h, w = frame_bgr.shape[:2]
            self.undistorter = Undistorter(
                self.K, self.dist, raw_size=(w, h),
                alpha=self.alpha, new_size=self.new_size
            )

    # ---------- UNDISTORTED path ----------

    def _prep_undistorted_gray_and_K(self, undist_bgr):
        gray = cv.cvtColor(undist_bgr, cv.COLOR_BGR2GRAY)
        K_work = self.undistorter.K_new.copy()
        dist_work = self.undistorter.dist_zero

        # Optional: crop to valid ROI from getOptimalNewCameraMatrix
        x, y, w, h = self.undistorter.roi
        if w > 0 and h > 0:
            gray = gray[y:y + h, x:x + w]
            K_work[0, 2] -= x
            K_work[1, 2] -= y

        # Apply user ops (crop/recenter/preprocess)
        gray, K_work = self._apply_user_ops(gray, K_work)
        return gray, K_work, dist_work

    # ---------- DISTORTED path ----------

    def _prep_distorted_gray_and_K(self, frame_bgr):
        gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)
        K_work = self.K.copy()
        dist_work = self.dist.copy()

        # Apply user ops (crop/recenter/preprocess)
        gray, K_work = self._apply_user_ops(gray, K_work)
        return gray, K_work, dist_work

    # ---------- Main entry ----------

    def process_bgr_frame(self, frame_bgr, t_cap, save_path=None, frame_counter=0, time_start_ref=0.0):
        """
        Input: distorted BGR frame.
        Output: (pose_dict, time_frame_captured, saved_path or None).
        """
        time_process = time.time()

        if getattr(config, "UNDISTORT", True):
            self._ensure_undistorter(frame_bgr)
            undist_bgr = self.undistorter.undistort(frame_bgr)
            gray, K_work, dist_work = self._prep_undistorted_gray_and_K(undist_bgr)
        else:
            gray, K_work, dist_work = self._prep_distorted_gray_and_K(frame_bgr)

        # Detect ArUco
        corners, ids, _ = self.detector.detectMarkers(gray)

        pose = {
            'x ArUco [m]': None,
            'y ArUco [m]': None,
            'z ArUco [m]': None,
            'roll ArUco [deg]': None,
            'pitch ArUco [deg]': None,
            'yaw ArUco [deg]': None,
        }

        annotated = gray
        found = False

        if ids is not None and config.ARUCO_ID in ids:
            idx = np.where(ids == config.ARUCO_ID)[0][0]
            image_points = corners[idx][0]
            s = float(config.ARUCO_REAL_SIZE)
            object_points = np.array(
                [[-s / 2,  s / 2, 0],
                 [ s / 2,  s / 2, 0],
                 [ s / 2, -s / 2, 0],
                 [-s / 2, -s / 2, 0]], dtype=np.float32
            )

            flag = cv.SOLVEPNP_IPPE_SQUARE if getattr(config, "SOLVER", 1) in (1, 2) else cv.SOLVEPNP_ITERATIVE
            if getattr(config, "SOLVER", 1) == 2 and self.prev_tvec is not None:
                success, rvec, tvec = cv.solvePnP(
                    object_points, image_points, K_work, dist_work,
                    rvec=self.prev_rvec, tvec=self.prev_tvec, useExtrinsicGuess=True, flags=flag
                )
            else:
                success, rvec, tvec = cv.solvePnP(object_points, image_points, K_work, dist_work, flags=flag)

            if success:
                self.prev_rvec, self.prev_tvec = rvec, tvec

                x, y, z = np.round(tvec.reshape(-1), 2)
                R, _ = cv.Rodrigues(rvec)
                sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
                if sy > 1e-6:
                    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
                    pitch = np.degrees(np.arctan2(-R[2, 0], sy))
                    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
                else:
                    yaw = np.degrees(np.arctan2(-R[0, 1], R[1, 1])); pitch = np.degrees(np.arctan2(-R[2, 0], sy)); roll = 0.0
                yaw, pitch, roll = np.round([yaw, pitch, roll], 1)

                pose.update({
                    'x ArUco [m]': x,
                    'y ArUco [m]': y,
                    'z ArUco [m]': z,
                    'roll ArUco [deg]': roll,
                    'pitch ArUco [deg]': pitch,
                    'yaw ArUco [deg]': yaw,
                })

                # Project & draw in the same domain we detected in
                center_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
                proj_center, _ = cv.projectPoints(center_3d, rvec, tvec, K_work, dist_work)
                pt = tuple(proj_center[0][0].astype(int))

                annotated = draw_on_frame(gray, pt)
                found = True

        duration_process = time.time() - time_process

        # Save exactly what we drew on
        time_save = time.time()
        saved_path = None
        if (save_path is not None) and (config.SAVE_IMAGES == 1 or config.SAVE_IMAGES == 2):
            os.makedirs(save_path, exist_ok=True)
            if found:
                fname = f"{save_path}/frame_{frame_counter:04d}_found_{(t_cap - time_start_ref):.3f}.png"
            else:
                fname = f"{save_path}/frame_{frame_counter:04d}_{(t_cap - time_start_ref):.3f}.png"
            if config.SAVE_IMAGES == 1:
                cv.imwrite(fname, frame_bgr)
            elif config.SAVE_IMAGES == 2:
                cv.imwrite(fname, annotated)
            saved_path = fname

        return pose, duration_process, saved_path


# =========================
# Helpers
# =========================

def draw_on_frame(frame, projected_point, corners=None):
    """
    Draw visualization on the frame:
    - Circle the detected ArUco tag with pink outline.
    - Draw a hollow circle around the projected marker center.
      The circle size adapts to the tag size.
    """
    if len(frame.shape) == 2:
        img = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
    else:
        img = frame.copy()
    
    # --- Hollow circle for marker center ---
    marker_radius = 50  # default
    cv.circle(img, projected_point, marker_radius, (0, 255, 255), 2)  # yellow outline, adaptive size

    return img

def capture_bgr_frame(picam2):
    """
    Capture a frame from Picamera2 and return it as a BGR numpy array
    (ready for OpenCV).
    """
    frame_rgb = picam2.capture_array("main")
    frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
    return frame_bgr


# =========================
# Camera + workflows
# =========================

def camera_init():
    """
    Initialize the camera (Picamera2).
    """
    picam2 = Picamera2()
    # NOTE: keep your (height,width) order from config to avoid breaking expectations.
    config_cam = picam2.create_video_configuration(
        main={"size": (config.CAM_HEIGHT_LOW, config.CAM_WIDTH_LOW)},
        raw={"size": (config.CAM_HEIGHT, config.CAM_WIDTH)}
    )
    picam2.configure(config_cam)

    if getattr(config, "AUTO_EXPOSURE", True):
        picam2.set_controls({"AwbEnable": True, "AeEnable": True})
    else:
        set_exposure(picam2, config.EXPOSURE_TIME, config.ANALOGUE_GAIN)
    picam2.start()
    meta = picam2.capture_metadata()
    print("Exposure time (µs):", meta.get("ExposureTime", "N/A"))
    return picam2

def set_exposure(picam2, exposure_time, analogue_gain):
    picam2.set_controls({
        "AwbEnable": False,
        "AeEnable": False,
        "ExposureTime": int(exposure_time),
        "AnalogueGain": float(analogue_gain)
    })
    time.sleep(0.1)  # allow time to adjust
    return True


def _stream_camera(picam, stop_event, window_name="Camera"):
    """
    OpenCV window preview (press 'q' to stop).
    """
    cv.namedWindow(window_name, cv.WINDOW_NORMAL)
    try:
        while not stop_event.is_set():
            try:
                frame_rgb = picam.capture_array()
                frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
                cv.imshow(window_name, frame_bgr)
            except Exception as e:
                print(f"[Stream] Capture error: {e}")
                break
            if cv.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break
    finally:
        cv.destroyWindow(window_name)


def calibrate_camera():
    """
    Checkerboard calibration.
    Saves images and 'camera_calib/calib_data/calib_data.npz' (mtx, dist, rms, image_size).
    Additionally creates:
      - camera_calib/result/before/ : 5 random raw images from the capture set
      - camera_calib/result/after/  : their undistorted counterparts (post-calibration)
    """
    print('Starting camera calibration...')

    # Base folders
    unann_dir = 'camera_calib/calib_images/unannotated'
    ann_dir = 'camera_calib/calib_images/annotated'
    data_dir = 'camera_calib/calib_data'
    result_dir = 'camera_calib/result'
    before_dir = os.path.join(result_dir, 'before')
    after_dir = os.path.join(result_dir, 'after')

    os.makedirs(unann_dir, exist_ok=True)
    os.makedirs(ann_dir, exist_ok=True)
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(before_dir, exist_ok=True)
    os.makedirs(after_dir, exist_ok=True)

    picam = camera_init()
    # Preview in background
    print("Starting live preview (press 'q' to close the preview window)...")
    stop_event = threading.Event()
    th = threading.Thread(target=_stream_camera, args=(picam, stop_event), daemon=True)
    th.start()

    # Capture stills
    print('Capturing calibration images...')
    time.sleep(2)
    try:
        for i in range(config.NB_IMAGES_CALIBRATION):
            print(f'Taking image {i+1}/{config.NB_IMAGES_CALIBRATION}')
            path = f'{unann_dir}/{i:03d}.jpg'
            try:
                picam.capture_file(path)
            except Exception as e:
                print(f"Failed to capture {path}: {e}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Capture interrupted by user.")
    finally:
        stop_event.set()
        th.join(timeout=2)

    # -------------------------
    # Pick 5 random "before" images
    # -------------------------
    SAMPLE_SEED = 12345
    rnd = random.Random(SAMPLE_SEED)

    all_imgs = sorted(glob.glob(os.path.join(unann_dir, '*.jpg')))
    if not all_imgs:
        print("No images found for calibration.")
    # select up to 5 distinct images
    k = min(5, len(all_imgs))
    sample_before = rnd.sample(all_imgs, k) if k > 0 else []

    # Copy raw images into result/0_before (keep original filenames)
    for src in sample_before:
        img = cv.imread(src)
        if img is None:
            print(f"[before] Skipping unreadable file: {src}")
            continue
        dst = os.path.join(before_dir, os.path.basename(src))
        cv.imwrite(dst, img)

    # -------------------------
    # Corner detection
    # -------------------------
    objp = np.zeros((1, config.CHECKERBOARD_SHAPE[0] * config.CHECKERBOARD_SHAPE[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[
        0:config.CHECKERBOARD_SHAPE[0],
        0:config.CHECKERBOARD_SHAPE[1]
    ].T.reshape(-1, 2)
    objp *= config.CALIBRATION_SQUARE

    objpoints, imgpoints = [], []
    subpix = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    image_shape = None

    print('Detecting checkerboard corners...')
    for fname in all_imgs:
        image = cv.imread(fname)
        if image is None:
            print(f"Could not read image: {fname}")
            continue

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(
            gray, config.CHECKERBOARD_SHAPE,
            flags=cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_FAST_CHECK | cv.CALIB_CB_NORMALIZE_IMAGE
        )
        if ret:
            print(f"Checkerboard detected in {os.path.basename(fname)}")
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix)
            objpoints.append(objp)
            imgpoints.append(corners2)

            annotated = cv.drawChessboardCorners(image.copy(), config.CHECKERBOARD_SHAPE, corners2, ret)
            cv.imwrite(f'{ann_dir}/{os.path.basename(fname)}', annotated)

            if image_shape is None:
                image_shape = gray.shape[::-1]  # (width, height)
        else:
            print(f"Checkerboard NOT found in {os.path.basename(fname)}")

    # -------------------------
    # Calibration + "after" images
    # -------------------------
    if len(objpoints) < 1 or image_shape is None:
        print('Not enough valid images for calibration.')
    else:
        print('Running calibration...')
        rms, mtx, dist, _, _ = cv.calibrateCamera(objpoints, imgpoints, image_shape, None, None)
        np.savez(f'{data_dir}/calib_data.npz', mtx=mtx, dist=dist, rms=rms, image_size=image_shape)
        print('Calibration complete.')
        print(f'RMS reprojection error: {rms:.4f}')
        print(f'Camera matrix:\n{mtx}')
        print(f'Distortion coefficients:\n{dist.ravel()}')
        print(f"Saved to {data_dir}/calib_data.npz")

        # Create undistorted versions of the same 5 "before" images in result/0_after
        # Use K_new and remap for clean results
        if sample_before:
            # Determine size from the first sampled image
            test_img = cv.imread(sample_before[0])
            if test_img is not None:
                h, w = test_img.shape[:2]
                K_new, _ = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), alpha=0.0)
                map1, map2 = cv.initUndistortRectifyMap(mtx, dist, None, K_new, (w, h), cv.CV_32FC1)

                for src in sample_before:
                    img = cv.imread(src)
                    if img is None:
                        print(f"[after] Skipping unreadable file: {src}")
                        continue
                    undist = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR)
                    dst = os.path.join(after_dir, os.path.basename(src))
                    cv.imwrite(dst, undist)
                print(f"Saved {len(sample_before)} undistorted images to {after_dir}")
            else:
                print("Could not read a sampled 'before' image to create 'after' set.")

    # Cleanup camera
    try:
        Picamera2.close(picam)
    except Exception:
        try:
            picam.stop()
        except Exception:
            pass


def generate_markers():
    """Generate and save an ArUco marker image."""
    os.makedirs("camera_calib", exist_ok=True)
    aruco_dict = cv.aruco.getPredefinedDictionary(config.ARUCO_DICT)
    marker = cv.aruco.generateImageMarker(aruco_dict, config.ARUCO_ID, config.ARUCO_PIXEL_SIZE)

    # try to reconstruct dictionary name for filename
    dict_name = [name for name in dir(cv.aruco) if getattr(cv.aruco, name) == config.ARUCO_DICT and name.startswith("DICT_")]
    dict_str = dict_name[0] if dict_name else "UNKNOWN_DICT"

    filename = f"{dict_str}_ID_{config.ARUCO_ID}_SIZE_{config.ARUCO_PIXEL_SIZE}px.png"
    cv.imwrite(os.path.join("camera_calib", filename), marker)
    print(f"Marker saved to camera_calib/{filename}")


def load_calibration():
    """Load calibration (mtx, dist, image_size)."""
    data = np.load('camera_calib/calib_data/calib_data.npz')
    return data['mtx'], data['dist'], tuple(data['image_size'])


def take_picture():
    """Capture one still, (optionally) undistort it, and save."""
    mtx, dist, _ = load_calibration()
    picam2 = camera_init()
    time.sleep(0.1)

    frame_rgb = picam2.capture_array("main")
    frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)

    if getattr(config, "UNDISTORT", True):
        pipeline = ArucoPipeline(
            mtx, dist,
            alpha=getattr(config, "UNDISTORT_ALPHA", 0.0),
            new_size=getattr(config, "UNDISTORT_SIZE", None)
        )
        pipeline._ensure_undistorter(frame_bgr)
        out_img = pipeline.undistorter.undistort(frame_bgr)
    else:
        out_img = frame_bgr

    os.makedirs("data", exist_ok=True)
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"data/cam_view_{ts}.jpg"
    cv.imwrite(filename, out_img)
    print(f"Image saved as {filename}")

    try:
        Picamera2.close(picam2)
    except Exception:
        try:
            picam2.stop()
        except Exception:
            pass


def annotate_aruco_in_folder(folder_path, mtx, dist):
    """
    Process all images in a folder:
    - if UNDISTORT: undistort → detect → draw (undistorted domain)
    - else: detect → draw (distorted domain)
    Saves to '<folder>/annotated_output'.
    """
    out_dir = os.path.join(folder_path, "annotated_output")
    os.makedirs(out_dir, exist_ok=True)

    files = sorted(glob.glob(os.path.join(folder_path, "*.png")) +
                   glob.glob(os.path.join(folder_path, "*.jpg")) +
                   glob.glob(os.path.join(folder_path, "*.jpeg")))
    if not files:
        print("No images found.")
        return

    pipeline = ArucoPipeline(
        mtx, dist,
        alpha=getattr(config, "UNDISTORT_ALPHA", 0.0),
        new_size=getattr(config, "UNDISTORT_SIZE", None)
    )

    found_count = 0
    t0 = time.time()
    for i, path in enumerate(files, 1):
        img = cv.imread(path)
        if img is None:
            print(f"Skipping unreadable file: {path}")
            continue
        pose, _, _ = pipeline.process_bgr_frame(
            img, time.time(), save_path=out_dir, frame_counter=i, time_start_ref=t0
        )
        if pose['x ArUco [m]'] is not None:
            found_count += 1

    print(f"Detected {found_count} of {len(files)} images.")

    images_to_mp4(out_dir, fps=1/config.DT_MAIN)


def images_to_mp4(image_path, output_path=None, fps=20, pattern=("*.jpg", "*.png")):
    """
    Create an MP4 video from all images in a folder.

    Args:
        image_path (str or Path): folder containing images.
        output_path (str or Path): path for the output .mp4 file.
        fps (int): frames per second (default 20).
        pattern (tuple): glob patterns of image formats to include.
    """
    if output_path is None:
        output_path = image_path / "onboard_video.mp4"
    else:
        output_path = output_path / "onboard_video.mp4"

    # Collect files
    files = []
    for p in pattern:
        files.extend(image_path.glob(p))
    if not files:
        raise ValueError(f"No images found in {image_path}")

    # Natural sort: 1, 2, 10 instead of 1, 10, 2
    def natural_key(p: Path):
        return [int(t) if t.isdigit() else t.lower()
                for t in re.findall(r'\d+|\D+', p.stem)]
    files = sorted(files, key=natural_key)

    # Read first image for size
    first = cv2.imread(str(files[0]))
    if first is None:
        raise RuntimeError(f"Could not read first image: {files[0]}")
    height, width, _ = first.shape

    # Define video writer (H.264 codec in MP4 container)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # safer than 'H264' for portability
    output_path.parent.mkdir(parents=True, exist_ok=True)
    out = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))

    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"Skipping unreadable file: {f}")
            continue
        if img.shape[0:2] != (height, width):
            img = cv2.resize(img, (width, height))
        out.write(img)

    out.release()
    print(f"MP4 video saved to {output_path}")


def camera_live_detect():
    """
    Live detection loop (Ctrl+C to stop).
    Saves annotated frames to 'data/live' if SAVE_IMAGES is True.
    """
    mtx, dist, _ = load_calibration()
    picam2 = camera_init()

    pipeline = ArucoPipeline(
        mtx, dist,
        alpha=getattr(config, "UNDISTORT_ALPHA", 0.0),
        new_size=getattr(config, "UNDISTORT_SIZE", None)
    )

    save_dir = "data/live"
    os.makedirs(save_dir, exist_ok=True)
    t0 = time.time()
    i = 0

    try:
        while True:
            frame_rgb = picam2.capture_array("main")
            frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
            i += 1
            pose, _, _ = pipeline.process_bgr_frame(
                frame_bgr, time.time(), save_path=save_dir, frame_counter=i, time_start_ref=t0
            )
            if pose['x ArUco [m]'] is not None:
                print(f"[{i}] Pose: x={pose['x ArUco [m]']}, y={pose['y ArUco [m]']}, z={pose['z ArUco [m]']}, "
                      f"r={pose['roll ArUco [deg]']}, p={pose['pitch ArUco [deg]']}, y={pose['yaw ArUco [deg]']}")
            else:
                print(f"[{i}] Marker not found.")
    except KeyboardInterrupt:
        print("Stopping live detection.")
    finally:
        try:
            Picamera2.close(picam2)
        except Exception:
            try:
                picam2.stop()
            except Exception:
                pass


# =========================
# CLI
# =========================

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Choose: 'calibrate_camera', 'generate_markers', 'take_picture', 'annotate_aruco <folder>', or 'live_detect'")
        sys.exit(0)

    cmd = sys.argv[1]
    if cmd == "calibrate_camera":
        calibrate_camera()
    elif cmd == "generate_markers":
        generate_markers()
    elif cmd == "take_picture":
        take_picture()
    elif cmd == "annotate_aruco":
        if len(sys.argv) >= 3:
            folder = sys.argv[2]
            mtx, dist, _ = load_calibration()
            annotate_aruco_in_folder(folder, mtx, dist)
        else:
            print("Please provide the path to the folder containing images.")
    elif cmd == "live_detect":
        camera_live_detect()
    else:
        print(f"Unknown function: {cmd}")
