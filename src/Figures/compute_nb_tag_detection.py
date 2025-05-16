
from pathlib import Path
import sys
import time
from datetime import datetime
import cv2 as cv
import numpy as np
import glob
import os

import config
import camera_ctrl


def detect_aruco(frame_rgb, mtx, dist, save_path, frame_counter):
    """
    Capture a frame and detect the specified ArUco marker.
    Returns rotation and translation vectors if found, else (None, None).
    Saves an annotated image showing the pose if detected.
    """
    gray = cv.cvtColor(frame_rgb, cv.COLOR_BGR2GRAY) # (gray scale format)
    filename = f"{save_path}/anotated_frame_{frame_counter:04d}.png"
    tag_detected = False

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
        img_center_x = frame_rgb.shape[1] / 2
        img_center_y = frame_rgb.shape[0] / 2
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
            tag_detected = True

            # Draw the marker and the center line on the frame
            annotated_frame = draw_on_frame(frame_rgb, projected_point)

            # Save annotated image
            cv.imwrite(filename, annotated_frame)
        else:
            # SolvePnP failed
            tag_detected = False

            # Save annotated image
            cv.imwrite(filename, frame_rgb)
    else:
        # ArUco not found
        tag_detected = False

        # Save annotated image
        cv.imwrite(filename, frame_rgb)

    return tag_detected    


def draw_on_frame(frame, projected_point):
    annotated_frame = frame.copy()
    height, width = annotated_frame.shape[:2]
    center_y = height // 2

    cv.line(annotated_frame, (0, center_y), (width, center_y), (147, 20, 255), 5)                                                               # draw center line
    cv.circle(annotated_frame, projected_point, 5, (255, 0, 0), -1)                                                                             # draw dot
    cv.putText(annotated_frame, "Marker center", (projected_point[0] + 10, projected_point[1]), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)   # draw text

    return annotated_frame



if __name__ == "__main__":
    save_path = 'Figures/frames_2025-05-13_14-51-29'
    mtx, dist = camera_ctrl.load_calibration()
    counter = 0
    frame_nb = 0
    nb_possible_detected = 150
    
    for frames in glob.glob(f"{save_path}/frame_*.png"):
        frame = cv.imread(frames)
        frame_nb += 1

        detected_tag = detect_aruco(frame, mtx, dist, save_path, frame_nb)

        if detected_tag:
            counter += 1
    
    print(f"Number of frames with detected tag: {counter}")
    print(f"Detection ratio: {counter/nb_possible_detected}")
