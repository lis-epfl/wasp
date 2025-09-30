from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
from libcamera import controls

from pathlib import Path
import threading
import sys
import time
from datetime import datetime
import cv2 as cv
import numpy as np
import glob
import os

import config

def camera_init():
    """
    Initialize the camera settings.
    :return picam2: Picamera2 object
    """
    picam2 = Picamera2()
    
    # Lower resolution for faster processing
    config_cam = picam2.create_video_configuration(main={"size": (config.CAM_HEIGHT_LOW, config.CAM_WIDTH_LOW)}, raw={"size": (config.CAM_HEIGHT, config.CAM_WIDTH)})
    picam2.configure(config_cam)

    # Manually set exposure
    if config.AUTO_EXPOSURE:
        picam2.set_controls({"AwbEnable": True, "AeEnable": True})  # Enable auto-exposure
    else:
        picam2.set_controls({
            "AwbEnable": False,                         # Disabling auto white balance
            "AeEnable": False,                          # Auto-exposure off
            "ExposureTime": config.EXPOSURE_TIME,       # Exposure time in microseconds
            "AnalogueGain": config.ANALOGUE_GAIN        # Fix gain for brightness
        })
    picam2.start()
    metadata = picam2.capture_metadata()
    print("Exposure time (µs):", metadata["ExposureTime"])

    return picam2


def _stream_camera(picam, stop_event, window_name="Camera"):
    """
    Background preview thread (OpenCV window). No Qt/GL preview, so no extra event loop.
    Press 'q' while the window is focused to stop the preview.
    """
    cv.namedWindow(window_name, cv.WINDOW_NORMAL)
    try:
        while not stop_event.is_set():
            try:
                frame_rgb = picam.capture_array()  # Picamera2 yields RGB
                frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
                cv.imshow(window_name, frame_bgr)
            except Exception as e:
                print(f"[Stream] Capture error: {e}")
                break

            # Check keypress
            if cv.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break
    finally:
        cv.destroyWindow(window_name)


def calibrate_camera():
    """
    Calibrate the camera using a checkerboard pattern.
    Captures images, detects corners, runs calibration, and saves results to:
      - Unannotated images: camera_calib/calib_images/unannotated/
      - Annotated images:   camera_calib/calib_images/annotated/
      - Calibration data:   camera_calib/calib_data.npz
    """
    print('Starting camera calibration...')

    # Setup camera
    picam = camera_init()

    # Ensure directories exist
    os.makedirs('camera_calib/calib_images/unannotated', exist_ok=True)
    os.makedirs('camera_calib/calib_images/annotated', exist_ok=True)
    os.makedirs('camera_calib/calib_data', exist_ok=True)

    # Start preview stream in background (no GUI event-loop conflicts)
    print("Starting live preview (press 'q' to close the preview window)...")
    stop_event = threading.Event()
    stream_thread = threading.Thread(target=_stream_camera, args=(picam, stop_event), daemon=True)
    stream_thread.start()

    # Capture images
    print('Capturing calibration images...')
    time.sleep(2)  # short settle time

    try:
        for i in range(config.NB_IMAGES_CALIBRATION):
            print(f'Taking image {i+1}/{config.NB_IMAGES_CALIBRATION}')
            filepath = f'camera_calib/calib_images/unannotated/{i:03d}.jpg'
            try:
                picam.capture_file(filepath)
            except Exception as e:
                print(f"Failed to capture {filepath}: {e}")
            time.sleep(0.5)  # brief pause between shots (adjust as needed)
    except KeyboardInterrupt:
        print("Capture interrupted by user.")
    finally:
        # Stop preview thread
        stop_event.set()
        stream_thread.join(timeout=2)

    # Prepare object points for checkerboard
    # CHECKERBOARD_SHAPE is expected as (cols, rows) = inner corners per row & per column
    objp = np.zeros(
        (1, config.CHECKERBOARD_SHAPE[0] * config.CHECKERBOARD_SHAPE[1], 3),
        np.float32
    )
    objp[0, :, :2] = np.mgrid[
        0:config.CHECKERBOARD_SHAPE[0],
        0:config.CHECKERBOARD_SHAPE[1]
    ].T.reshape(-1, 2)
    objp *= config.CALIBRATION_SQUARE  # square size in meters

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    subpix_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    image_shape = None

    # Process captured images
    print('Detecting checkerboard corners...')
    images = sorted(glob.glob('camera_calib/calib_images/unannotated/*.jpg'))
    for fname in images:
        image = cv.imread(fname)
        if image is None:
            print(f"Could not read image: {fname}")
            continue

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        ret, corners = cv.findChessboardCorners(
            gray,
            config.CHECKERBOARD_SHAPE,
            flags=cv.CALIB_CB_ADAPTIVE_THRESH |
                  cv.CALIB_CB_FAST_CHECK |
                  cv.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret:
            print(f"Checkerboard detected in {os.path.basename(fname)}")
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix_criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)

            annotated = cv.drawChessboardCorners(image.copy(), config.CHECKERBOARD_SHAPE, corners2, ret)
            annotated_fname = f'camera_calib/calib_images/annotated/{os.path.basename(fname)}'
            try:
                cv.imwrite(annotated_fname, annotated)
            except Exception as e:
                print(f"Failed to write annotated image {annotated_fname}: {e}")

            if image_shape is None:
                image_shape = gray.shape[::-1]  # (width, height)
        else:
            print(f"Checkerboard NOT found in {os.path.basename(fname)}")

    # Calibration computation
    if len(objpoints) < 1:
        print('Not enough valid images for calibration.')
    else:
        print('Running calibration...')
        # retval is RMS reprojection error (float), not a boolean
        rms, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            objpoints, imgpoints, image_shape, None, None
        )

        # Save calibration
        try:
            np.savez('camera_calib/calib_data.npz', mtx=mtx, dist=dist, rms=rms, image_size=image_shape)
            print('Calibration complete.')
            print(f'RMS reprojection error: {rms:.4f}')
            print(f'Camera matrix:\n{mtx}')
            print(f'Distortion coefficients:\n{dist.ravel()}')
            print("Saved to camera_calib/calib_data.npz")
        except Exception as e:
            print(f"Failed to save calibration data: {e}")

    # Cleanup camera
    try:
        Picamera2.close(picam)  # newer API
    except Exception:
        try:
            picam.stop()
        except Exception:
            pass


def generate_markers():
    """
    Generate an ArUco tag and save it as an image in the camera_calib directory.
    """
    os.makedirs("camera_calib", exist_ok=True)
    
    aruco_dict = cv.aruco.getPredefinedDictionary(config.ARUCO_DICT)
    marker_img = cv.aruco.generateImageMarker(aruco_dict, config.ARUCO_ID, config.ARUCO_PIXEL_SIZE)

    # Get the dictionary name as string (reverse lookup)
    dict_name = [name for name in dir(cv.aruco) if getattr(cv.aruco, name) == config.ARUCO_DICT and name.startswith("DICT_")]
    dict_str = dict_name[0] if dict_name else "UNKNOWN_DICT"

    # Create filename with dictionary, ID, and size
    filename = f"{dict_str}_ID_{config.ARUCO_ID}_SIZE_{config.ARUCO_PIXEL_SIZE}px.png"

    # Save the marker image
    cv.imwrite(os.path.join("camera_calib", filename), marker_img)
    print(f"Marker saved to camera_calib/{filename}")


def load_calibration():
    """
    Load the calibration data.
    """
    data = np.load('camera_calib/calib_data.npz')
    mtx = data['mtx']
    dist = data['dist']

    return mtx, dist

_undistort_maps = None
def build_undistort_maps(image_shape, mtx, dist):
    global _undistort_maps
    if _undistort_maps is None or _undistort_maps[0].shape[::-1] != (image_shape[1], image_shape[0]):
        newcameramtx, _ = cv.getOptimalNewCameraMatrix(mtx, dist, (image_shape[1], image_shape[0]), 0)
        _undistort_maps = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx,
                                                     (image_shape[1], image_shape[0]), cv.CV_16SC2)
    return _undistort_maps

prev_tvec = None
prev_rvec = None
def detect_aruco_pose(picam2, mtx, dist, save_path, frame_counter, time_start_ref, post=False):
    """
    Capture a frame and detect the specified ArUco marker.
    :param picam2: Picamera2 object
    :param mtx: Camera matrix
    :param dist: Distortion coefficients
    :param save_path: Path to save the captured frame
    :param frame_counter: Frame counter for naming the saved file
    :return: Pose information (position and orientation) if found, else None and time when frame captured.
    """
    if not post:
        # Capture a frame
        frame_rgb = picam2.capture_array("main")
        time_frame_captured = time.time()
    else:
        # load image
        frame_rgb = picam2 # here picam2 an image
        time_frame_captured = time.time()

    frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
    gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)

    # Crop the image if specified (config.CROP_X is in percentage of width, config.CROP_Y is in percentage of height)
    if config.CROP_X > 0 or config.CROP_Y > 0:
        h, w = gray.shape[:2]
        x1 = int(w * config.CROP_X/2)
        y1 = int(h * config.CROP_Y/2)
        x2 = int(w * (1 - config.CROP_X/2))
        y2 = int(h * (1 - config.CROP_Y/2))
        gray = gray[y1:y2, x1:x2]

    # Adjust camera matrix to center the image (origin at the center of the image)
    img_center_x = frame_bgr.shape[1] / 2
    img_center_y = frame_bgr.shape[0] / 2
    adjusted_mtx = mtx.copy()

    if config.RECENTER_ORIGIN:
        adjusted_mtx[0, 2] = img_center_x
        adjusted_mtx[1, 2] = img_center_y

    if config.PRE_PROCESS:
        # Lower resolution
        sx = config.RES_DROP_PRE
        sy = config.RES_DROP_PRE
        gray = cv.resize(gray, (0,0), fx=sx, fy=sy, interpolation=cv.INTER_AREA)
        S = np.array([[sx, 0,  0],
                  [0,  sy, 0],
                  [0,  0,  1]], dtype=adjusted_mtx.dtype)
        adjusted_mtx = S @ adjusted_mtx

        # Local contrast enhancement
        clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(4,4))
        gray = clahe.apply(gray)

        # Sharpen
        blur = cv.GaussianBlur(gray, (0,0), 1.0)
        gray = cv.addWeighted(gray, 2, blur, -1, 0)

    # ArUco dictionary and detection setup
    dictionary = cv.aruco.getPredefinedDictionary(config.ARUCO_DICT)
    parameters = cv.aruco.DetectorParameters()

    if config.ADVANCED_PARAMETERS:
        # 1) Spend more time on thresholding (more tries, larger windows)
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 53      # ↑ from 23 for tougher lighting
        parameters.adaptiveThreshWinSizeStep = 4      # smaller step = more attempts
        parameters.adaptiveThreshConstant = 7

        # 2) Widen candidate search while staying sane
        parameters.minMarkerPerimeterRate = 0.01      # small markers
        parameters.maxMarkerPerimeterRate = 6.0       # ↑ allow very large candidates
        parameters.polygonalApproxAccuracyRate = 0.03 # 0.02–0.05; lower = tighter contour fit

        # 3) Be tolerant but not sloppy with borders/bits
        parameters.markerBorderBits = 1               # if your printed markers use 1; else match your print
        parameters.maxErroneousBitsInBorderRate = 0.5 # ↑ tolerate imperfect borders (default ~0.35)

        # 4) Corner refinement (already on) — make it a bit more tenacious
        parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 7        # was 5; 5–9 helps with blur/noise
        parameters.cornerRefinementMaxIterations = 100
        parameters.cornerRefinementMinAccuracy = 0.005

        # 5) Perspective normalization — give it more resolution to work with
        parameters.perspectiveRemovePixelPerCell = 10 # 8→10 or 12 increases detail
        parameters.perspectiveRemoveIgnoredMarginPerCell = 0.33

        # 6) Distance/spacing heuristics
        parameters.minCornerDistanceRate = 0.03       # ↓ allows tighter clusters of candidates
        parameters.minMarkerDistanceRate = 0.02       # ↓ helps when markers are close together

        # 7) Robustness toggles
        parameters.detectInvertedMarker = True
        
    detector = cv.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and config.ARUCO_ID in ids:
        # ArUco found
        filename = f"{save_path}/frame_{frame_counter:04d}_found_{(time_frame_captured-time_start_ref):.3f}.png"

        # Define 3D object points for the marker (centered at origin, Z=0)
        idx = np.where(ids == config.ARUCO_ID)[0][0]
        image_points = corners[idx][0]
        s = config.ARUCO_REAL_SIZE
        object_points = np.array([
            [-s/2,  s/2, 0],  # top-left
            [ s/2,  s/2, 0],  # top-right
            [ s/2, -s/2, 0],  # bottom-right
            [-s/2, -s/2, 0]   # bottom-left
        ], dtype=np.float32)

        # Solve PnP to find the rotation and translation vectors
        if config.SOLVER == 0:
            success, rvec, tvec = cv.solvePnP(object_points, image_points, adjusted_mtx, dist, flags=cv.SOLVEPNP_ITERATIVE)
        elif config.SOLVER == 1:
            success, rvec, tvec = cv.solvePnP(object_points, image_points, adjusted_mtx, dist, flags=cv.SOLVEPNP_IPPE_SQUARE)
        elif config.SOLVER == 2:
            global prev_tvec
            global prev_rvec
            if prev_tvec is None:
                success, rvec, tvec = cv.solvePnP(object_points, image_points, adjusted_mtx, dist, flags=cv.SOLVEPNP_IPPE_SQUARE)
            else:
                success, rvec, tvec = cv.solvePnP(
                    object_points, image_points, adjusted_mtx, dist,
                    rvec=prev_rvec, tvec=prev_tvec, useExtrinsicGuess=True,
                    flags=cv.SOLVEPNP_IPPE_SQUARE
                    )
            prev_tvec = tvec
            prev_rvec = rvec
            
        else:
            print("Unknown solver selected in config.SOLVER.")

        # Compute ArUco position and orientation
        x_aruco = np.round(tvec[0], 2)
        y_aruco = np.round(tvec[1], 2)
        z_aruco = np.round(tvec[2], 2)

        R, _ = cv.Rodrigues(rvec)  # rvec -> rotation matrix

        # Check for gimbal lock
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)

        if sy > 1e-6:  # not singular
            yaw_aruco   = np.degrees(np.arctan2(R[1,0], R[0,0]))
            pitch_aruco = np.degrees(np.arctan2(-R[2,0], sy))
            roll_aruco  = np.degrees(np.arctan2(R[2,1], R[2,2]))
        else:  # singular (gimbal lock)
            yaw_aruco   = np.degrees(np.arctan2(-R[0,1], R[1,1]))
            pitch_aruco = np.degrees(np.arctan2(-R[2,0], sy))
            roll_aruco  = 0

        # Round to 1 decimal place
        yaw_aruco   = np.round(yaw_aruco, 1)
        pitch_aruco = np.round(pitch_aruco, 1)
        roll_aruco  = np.round(roll_aruco, 1)

        # Project the 3D center of the marker to the image
        marker_center_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        projected_center, _ = cv.projectPoints(marker_center_3d, rvec, tvec, adjusted_mtx, dist)
        projected_point = tuple(projected_center[0][0].astype(int))

    else:
        # ArUco not found
        filename = f"{save_path}/frame_{frame_counter:04d}_{(time_frame_captured-time_start_ref):.3f}.png"

        x_aruco = None
        y_aruco = None
        z_aruco = None
        yaw_aruco = None
        pitch_aruco = None
        roll_aruco = None

    # Save image
    if config.SAVE_IMAGES:
        map1, map2 = build_undistort_maps(gray.shape, adjusted_mtx, dist)
        undistorted = cv.remap(gray, map1, map2, interpolation=cv.INTER_LINEAR)
        cv.imwrite(filename, undistorted)

    return {
        'x ArUco [m]': x_aruco,
        'y ArUco [m]': y_aruco,
        'z ArUco [m]': z_aruco,
        'roll ArUco [deg]': roll_aruco,
        'pitch ArUco [deg]': pitch_aruco,
        'yaw ArUco [deg]': yaw_aruco,
    }, time_frame_captured


def draw_on_frame(frame, projected_point):
    """
    Draw the center line and the projected point on the frame.
    :param frame: The original frame.
    :param projected_point: The 2D point to project.
    :return annotated_frame: The annotated frame with the center line and projected point.
    """
    annotated_frame = frame.copy()
    height, width = annotated_frame.shape[:2]
    center_x = width // 2

    cv.line(annotated_frame, (center_x, 0), (center_x, height), (147, 20, 255), 5)                                                              # draw center line
    cv.circle(annotated_frame, projected_point, 10, (255, 0, 0), -1)                                                                            # draw dot
    cv.putText(annotated_frame, "Marker center", (projected_point[0] + 10, projected_point[1]), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)   # draw text

    return annotated_frame


def take_picture():
    """
    Capture a still image, undistort it, and save it to a file.
    """
    mtx, dist = load_calibration()
    picam2 = camera_init()
    time.sleep(0.1)  # Give some time for the camera to adjust

    frame = picam2.capture_array("main")
    frame_bgr = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

    map1, map2 = build_undistort_maps(gray.shape, mtx, dist)
    undistorted = cv.remap(gray, map1, map2, interpolation=cv.INTER_LINEAR)
    # undistorted = frame_bgr.copy() # No undistortion = original image 

    save_path = "data"
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{save_path}/cam_view_{timestamp}.jpg"
    cv.imwrite(filename, undistorted)

    print(f"Image saved as {filename}")


def annotate_aruco_in_folder(folder_path, mtx, dist):
    """
    Process all images in a folder, detect the specified ArUco marker,
    draw the marker boundary if detected, and save annotated images
    in an automatically created subfolder called 'annotated_output'.
    """
    # Load ArUco dictionary and detector
    dictionary = cv.aruco.getPredefinedDictionary(config.ARUCO_DICT)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)

    # Create output folder
    output_folder = os.path.join(folder_path, "annotated_output")
    os.makedirs(output_folder, exist_ok=True)

    # Get all image files in the folder
    image_files = sorted(glob.glob(os.path.join(folder_path, "*.png")) +
                         glob.glob(os.path.join(folder_path, "*.jpg")) +
                         glob.glob(os.path.join(folder_path, "*.jpeg")))
    
    frame_counter = 0
    detection_counter = 0

    for image_path in image_files:
         
        frame_counter += 1

        image = cv.imread(image_path)
        if image is None:
            print(f"Skipping unreadable file: {image_path}")
            continue

        # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # corners, ids, _ = detector.detectMarkers(gray)

        # if ids is not None and config.ARUCO_ID in ids:
        #     idx = np.where(ids == config.ARUCO_ID)[0][0]
        #     image_points = corners[idx][0]  # shape (4,2), corner points in image

        #     # Define 3D object points for the marker (centered at origin, Z=0)
        #     s = config.ARUCO_REAL_SIZE
        #     object_points = np.array([
        #         [-s/2,  s/2, 0],  # top-left
        #         [ s/2,  s/2, 0],  # top-right
        #         [ s/2, -s/2, 0],  # bottom-right
        #         [-s/2, -s/2, 0]   # bottom-left
        #     ], dtype=np.float32)

        #     # Adjust camera matrix to center the image (origin at the center of the image)
        #     img_center_x = image.shape[1] / 2
        #     img_center_y = image.shape[0] / 2
        #     adjusted_mtx = mtx.copy()
        #     adjusted_mtx[0, 2] = img_center_x
        #     adjusted_mtx[1, 2] = img_center_y

        #     # Solve PnP to find the rotation and translation vectors
        #     success, rvec, tvec = cv.solvePnP(
        #         object_points,
        #         image_points,
        #         adjusted_mtx,
        #         dist,
        #         flags=cv.SOLVEPNP_ITERATIVE
        #         )

        #     # Project the 3D center of the marker to the image
        #     marker_center_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        #     projected_center, _ = cv.projectPoints(marker_center_3d, rvec, tvec, adjusted_mtx, dist)
        #     projected_point = tuple(projected_center[0][0].astype(int))

        #     if success:
        #         annotated_image = draw_on_frame(image, projected_point)
        # else:
        #     print(f"No marker found in: {image_path}")
        #     annotated_image = image.copy()

        # # Save to annotated_output
        # filename = os.path.basename(image_path)
        # filename = f"{output_folder}/frame_{frame_counter:04d}.png"
        # cv.imwrite(filename, annotated_image)
        # print(f"Annotated and saved: {filename}")

        dictionary, timeframe = detect_aruco_pose(image, mtx, dist, output_folder, frame_counter, time_start_ref=0, post=True)
        if dictionary['x ArUco [m]'] is not None:
            detection_counter += 1
    print(f"Detected {detection_counter} of {frame_counter} images.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Choose either 'calibrate_camera', 'generate_markers', 'take_video', 'take_picture', or 'annotate_aruco'")
    elif sys.argv[1] == "calibrate_camera":
        calibrate_camera()
    elif sys.argv[1] == "generate_markers":
        generate_markers()
    elif sys.argv[1] == "take_video":
        take_video()
    elif sys.argv[1] == "take_picture":
        take_picture()
    elif sys.argv[1] == "annotate_aruco":
        if len(sys.argv) >= 3:
            folder_path = sys.argv[2]
            mtx, dist = load_calibration()
            annotate_aruco_in_folder(folder_path, mtx, dist)
        else:
            print("Please provide the path to the folder containing images.")
    else:
        print(f"Unknown function: {sys.argv[1]}")
