from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
from pathlib import Path

import sys
import time
from datetime import datetime
import cv2 as cv
import numpy as np
import glob
import os

import config


def calibrate_camera():
    """
    Calibrate the camera using a checkerboard pattern.
    This function captures images of the checkerboard pattern and computes the camera matrix and distortion coefficients.
    """
    print('Starting camera calibration...')

    # Setup camera
    picam = Picamera2()
    picam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
    picam.start()
    time.sleep(1)

    # Ensure directories exist
    os.makedirs('camera_calib/calib_images/unannotated', exist_ok=True)
    os.makedirs('camera_calib/calib_images/annotated', exist_ok=True)
    os.makedirs('camera_calib/calib_data', exist_ok=True)

    # Capture images
    print('Capturing calibration images...')
    for i in range(config.NB_IMAGES_CALIBRATION):
        print(f'Taking image {i}')
        filepath = f'camera_calib/calib_images/unannotated/{i}.jpg'
        picam.capture_file(filepath)
        time.sleep(4) # Wait for a few seconds before taking the next picture

    picam.stop()

    # Prepare object points
    objp = np.zeros((1, config.CHECKERBOARD_SHAPE[0] * config.CHECKERBOARD_SHAPE[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:config.CHECKERBOARD_SHAPE[0], 0:config.CHECKERBOARD_SHAPE[1]].T.reshape(-1, 2)
    objp *= config.CALIBRATION_SQUARE # in meters

    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane.

    subpix_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1) # Stop when either 30 iterations are done or the corner pos are not changing significantly (less than 0.1 px change)
    image_shape = None

    # Process captured images
    print('Detecting checkerboard corners...')
    images = glob.glob('camera_calib/calib_images/unannotated/*.jpg')
    for fname in images:
        image = cv.imread(fname)
        if image is None:
            print(f"Could not read image: {fname}")
            continue

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, config.CHECKERBOARD_SHAPE, 
                                                cv.CALIB_CB_ADAPTIVE_THRESH + 
                                                cv.CALIB_CB_FAST_CHECK + 
                                                cv.CALIB_CB_NORMALIZE_IMAGE)
        if ret:
            print(f"Checkerboard in {fname}")
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), subpix_criteria) # search window size 11x11, no zero zone with (-1, -1)
            objpoints.append(objp)
            imgpoints.append(corners2)

            annotated = cv.drawChessboardCorners(image, config.CHECKERBOARD_SHAPE, corners2, ret)
            annotated_fname = f'camera_calib/calib_images/annotated/{os.path.basename(fname)}'
            cv.imwrite(annotated_fname, annotated)

            if image_shape is None:
                image_shape = gray.shape[::-1]
        else:
            print(f"Checkerboard not found in {fname}")

    # Calibration computation
    if len(objpoints) < 1:
        print('Not enough valid images for calibration.')
        return

    print('Running calibration...')
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, image_shape, None, None)

    if ret:
        np.savez('camera_calib/calib_data.npz', mtx=mtx, dist=dist)
        print('Calibration complete.')
        print(f'Camera matrix:\n{mtx}')
        print(f'Distortion coefficients:\n{dist}')
    else:
        print('Calibration failed.')


def generate_markers():
    '''
    Generate an ArUco tag and save it as an image
    '''
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
    '''
    Load the calibration data.
    '''
    data = np.load('camera_calib/calib_data.npz')
    mtx = data['mtx']
    dist = data['dist']

    return mtx, dist


def camera_init():
    """
    Initialize the Picamera2.
    """
    picam2 = Picamera2()
    # Lower resolution for faster processing
    config_cam = picam2.create_video_configuration(main={"size": (config.CAM_HEIGHT_LOW, config.CAM_WIDTH_LOW)},
                                                   raw={"size": (config.CAM_HEIGHT, config.CAM_WIDTH)})
    picam2.configure(config_cam)
    picam2.start()
    return picam2


def detect_aruco_pose(picam2, mtx, dist, save_path, frame_counter):
    """
    Capture a frame and detect the specified ArUco marker.
    Returns rotation and translation vectors if found, else (None, None).
    Saves an annotated image showing the pose if detected.
    """

    # Capture a frame
    frame_rgb = picam2.capture_array("main")  # Non-blocking read of latest frame (rgb format)
    frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR) # (bgr format)
    gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY) # (gray scale format)
    filename = f"{save_path}/frame_{frame_counter:04d}.png"

    # ArUco dictionary and detection setup
    dictionary = cv.aruco.getPredefinedDictionary(config.ARUCO_DICT)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and config.ARUCO_ID in ids:
        idx = np.where(ids == config.ARUCO_ID)[0][0]
        image_points = corners[idx][0]  # shape (4,2), corner points in image

        # Define 3D object points for the marker (centered at origin, Z=0)
        s = config.ARUCO_REAL_SIZE
        object_points = np.array([
            [-s/2,  s/2, 0],  # top-left
            [ s/2,  s/2, 0],  # top-right
            [ s/2, -s/2, 0],  # bottom-right
            [-s/2, -s/2, 0]   # bottom-left
        ], dtype=np.float32)

        # Adjust camera matrix to center the image (origin at the center of the image)
        img_center_x = frame_bgr.shape[1] / 2
        img_center_y = frame_bgr.shape[0] / 2
        adjusted_mtx = mtx.copy()
        adjusted_mtx[0, 2] = img_center_x
        adjusted_mtx[1, 2] = img_center_y

        # Solve PnP to find the rotation and translation vectors
        success, rvec, tvec = cv.solvePnP(
            object_points,
            image_points,
            adjusted_mtx,
            dist,
            flags=cv.SOLVEPNP_ITERATIVE
            )

        # Project the 3D center of the marker to the image
        marker_center_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        projected_center, _ = cv.projectPoints(marker_center_3d, rvec, tvec, adjusted_mtx, dist)
        projected_point = tuple(projected_center[0][0].astype(int))

        if success:
            # SolvePnP succeeded
            offset_from_center = tvec[1]

            # Draw the marker and the center line on the frame
            annotated_frame = frame_bgr.copy()
            height, width = annotated_frame.shape[:2]
            center_y = height // 2
            cv.line(annotated_frame, (0, center_y), (width, center_y), (147, 20, 255), 5) # draw center line
            cv.circle(annotated_frame, projected_point, 5, (255, 0, 0), -1)  # draw dot
            cv.putText(annotated_frame, "Marker center", (projected_point[0] + 10, projected_point[1]),
            cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2) # draw text

            # Save annotated image
            cv.imwrite(filename, annotated_frame)
        else:
            # SolvePnP failed
            offset_from_center = None

            # Save annotated image
            cv.imwrite(filename, frame_bgr)
    else:
        # ArUco not found
        offset_from_center = None

        # Save annotated image
        cv.imwrite(filename, frame_bgr)

    return offset_from_center    


def take_video():
    """
    Record video and save it to a file.
    """
    # Initialize the camera
    picam2 = Picamera2()

    # Configure the camera for video recording
    video_config = picam2.create_video_configuration()
    picam2.configure(video_config)
    picam2.set_controls({"FrameRate": config.FRAME_RATE})

    # Generate a unique filename with timestamp
    save_path = "data"
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{save_path}/video_{timestamp}.mp4"

    # Initialize the encoder and output
    encoder = H264Encoder()
    output = FfmpegOutput(filename)

    try:
        print(f"Recording started: {filename}")
        picam2.start_recording(encoder, output, quality=Quality.HIGH)

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping recording...")
        picam2.stop_recording()
        print(f"Video saved as {filename}")


def take_picture():
    """
    Capture a still image and save it to a file.
    """
    picam2 = camera_init()
    time.sleep(2)  # Give some time for the camera to adjust
    frame = picam2.capture_array("main")  # Non-blocking read of latest frame

    save_path = "data"
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{save_path}/cam_view{timestamp}.jpg"
    picam2.capture_file(filename)
    print(f"Image saved as {filename}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Choose either 'calibrate_camera', 'generate_markers', 'take_video', or 'take_picture'")
    elif sys.argv[1] == "calibrate_camera":
        calibrate_camera()
    elif sys.argv[1] == "generate_markers":
        generate_markers()
    elif sys.argv[1] == "take_video":
        take_video()
    elif sys.argv[1] == "take_picture":
        take_picture()
    else:
        print(f"Unknown function: {sys.argv[1]}")

# if __name__ == "__main__":
#     picam2 = camera_init()
#     try:
#         while True:
#             time_start = time.time()
#             offset_from_center = detect_aruco_pose(picam2)
#             print(f"Offset from center: {offset_from_center}")
#             time_end = time.time()
#             elapsed_time = time_end - time_start
#             print(f"Elapsed time: {elapsed_time:.2f} seconds") # this takes about 0.3 seconds
#     except KeyboardInterrupt:
#         print("Exiting...")
#     finally:
#         picam2.stop()
