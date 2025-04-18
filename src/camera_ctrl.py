from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
import sys
import time
from datetime import datetime
import cv2 as cv
import numpy as np
import glob
import os
from libcamera import controls

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
        time.sleep(2)

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
    Initialize the camera and configure it for video recording.
    """
    # Initialize the camera
    picam2 = Picamera2()

    # Configure the camera for video recording
    video_config = picam2.create_video_configuration()
    picam2.configure(video_config)
    picam2.set_controls({"FrameRate": config.FRAME_RATE})

    return picam2








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
    # Initialize the camera
    picam2 = Picamera2()

    # Configure the camera for still image capture
    camera_config = picam2.create_still_configuration()
    picam2.configure(camera_config)

    # Start the camera
    picam2.start()
    time.sleep(2)  # Give some time for the camera to adjust

    save_path = "data"
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{save_path}/cam_view{timestamp}.jpg"

    # Capture and save the image
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