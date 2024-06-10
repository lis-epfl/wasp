import cv2 as cv
import os
import glob
import argparse
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
import time

import config



class Detector:

    def __init__(self,dictionary,aruco_id,aruco_size,real_size):
        self.dictionary = dictionary
        self.aruco_id = aruco_id
        self.aruco_size = aruco_size
        self.real_size = real_size
        self.parameters = cv.aruco.DetectorParameters()
        self.mtx = None
        self.dist = None

    def calibration(self):
        '''
        Calibration of the camera
        '''
        print('Begin calibration')
        picam = Picamera2()
        picam.set_controls({
        "AfMode":controls.AfModeEnum.Continuous,

        })
        picam.start()
        time.sleep(1)
        for i in range(10):
            print('Capture')
            picam.capture_file('calibration_images/unannotated/'+str(i)+'.jpg')
            print('Move')
            time.sleep(2)
        data = config.load('config.json')
        square_size = data['aruco']['calibration_square']
        CHECKERBOARD = (5,8)
        subpix_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)

        three_d_points = []
        two_d_points = []

        object_p3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        object_p3d[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        object_p3d *= square_size

        prev_img_shape = None

        images = glob.glob('calibration_images/unannotated/*.jpg')

        for fname in images:
            image = cv.imread(fname)
            if image is None:
                print("Failed to read image:", fname)
                continue
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, 
                                                    cv.CALIB_CB_ADAPTIVE_THRESH + 
                                                    cv.CALIB_CB_FAST_CHECK + 
                                                    cv.CALIB_CB_NORMALIZE_IMAGE)
            if ret:
                print("Chessboard detected!")
                three_d_points.append(object_p3d)
                corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), subpix_criteria)
                two_d_points.append(corners2)

                image = cv.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)
                if prev_img_shape == None:
                    prev_img_shape = gray.shape
                else:
                    assert prev_img_shape == gray.shape
                # Save the image
                cv.imwrite('calibration_images/annotated/' + os.path.basename(fname), image)
            else:
                print("Chessboard not detected in image:", fname)
        if len(three_d_points) > 0 and len(two_d_points) > 0:
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(three_d_points, two_d_points, 
                                                              gray.shape[::-1], None, None)
        else:
            print("Insufficient data for calibration.")
        
        if ret:
            np.savez('calibration_data/calibration.npz', mtx=mtx, dist=dist)
        else:
            print("Calibration failed.")

    def load_calibration(self):
        '''
        Load the calibration data
        '''
        data = np.load('calibration_data/calibration.npz')
        self.mtx = data['mtx']
        self.dist = data['dist']

    def generate_markers(self):
        '''
        Generate a ArUco tag for the robot to follow
        '''
        tag = cv.aruco.generateImageMarker(self.dictionary, self.aruco_id, self.aruco_size)
        cv.imwrite("tag.png", tag)


    def detect_markers(self,frame):
        '''
        Detect the ArUco tag in the frame
        '''
        marker_corners = []
        marker_ids = []
        rejected_candidates = []
        detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(frame)

        return marker_corners, marker_ids, rejected_candidates

    def draw_markers(self,frame, corners):
        '''
        Draw the ArUco tag on the frame
        '''
        cv.aruco.drawDetectedMarkers(frame, corners)
        return frame


    def get_marker_center(self,corners):
        '''
        Get the center of the ArUco tag
        '''
        corners = corners[0]
        x = 0
        y = 0   
        for corner in corners:
            x += corner[0]
            y += corner[1]
        x = x / 4
        y = y / 4
        return (int(x), int(y))
    
    def pixel_to_meter(self, pixel_distance, tvecs):
        # Focal length in pixels
        focal_length_px = (self.mtx[0, 0] + self.mtx[1, 1]) / 2
        # Distance from the camera to the tag in meters (Z component of tvecs)
        distance_to_tag_m = tvecs[2]
        # Real-world size of the ArUco marker (in meters)
        marker_size_m = self.real_size
        # Size of the ArUco marker in pixels (from the detected corners)
        marker_size_px = self.aruco_size

        # Calculate the size of one pixel in meters
        pixel_size_m = marker_size_m / marker_size_px
        # Convert pixel distance to meters
        distance_m = pixel_distance * pixel_size_m

        return distance_m


    def marker_position(self, corners):
        rvect = []
        tvect = []
        marker_points = np.array([
        [-self.real_size / 2, self.real_size / 2, 0],
        [self.real_size / 2, self.real_size / 2, 0],
        [self.real_size / 2, -self.real_size / 2, 0],
        [-self.real_size / 2, -self.real_size / 2, 0]
    ], dtype=np.float32)
        for c in corners:
            _, rvec, tvec = cv.solvePnP(marker_points, c, self.mtx, self.dist, False, cv.SOLVEPNP_IPPE_SQUARE)
            rvect.append(rvec)
            tvect.append(tvec)
        return rvect, tvect

    def draw_center(self,frame, center):
        '''
        Draw the center of the ArUco tag on the frame
        '''

        cv.circle(frame, center, 5, (0, 0, 255), -1)
        return frame

    def markers_detection(self,frame,draw=False):
        '''
        Detect the wanted ArUco tag in the frame and draw the center of the tag if draw is True.
        Calculate the rotation vector and the translation vector of the tag.
        Returns the frame with the center of the tag drawn and the different vectors.\n
    params:
        - frame: the frame to detect the tag
        - draw: if True, draw the center of the tag on the frame\n
        return:
        - drawframe: the frame with the center of the tag drawn
        - rvecs: the rotation vector of the tag
        - tvecs: the translation vector of the tag
        '''
        corners, ids, _ = self.detect_markers(frame)
        center = None
        drawframe = None
        detected = False
        if ids is not None:
            for i in range(len(ids)):
                if ids[i] == self.aruco_id:
                    detected = True
                    h, w, _ = frame.shape
                    frame_center = (w / 3, h / 2)                    
                    center = self.get_marker_center(corners[i])
                    delta_x_pixels = center[0] - frame_center[0]
                    delta_y_pixels = center[1] - frame_center[1]
                    rvecs, tvecs = self.marker_position(corners[i])
                    rvecs, tvecs = rvecs[i], tvecs[i]
                    delta_x_m = self.pixel_to_meter(delta_x_pixels, tvecs)
                    delta_y_m = self.pixel_to_meter(delta_y_pixels, tvecs)
                    distance_from_center_m = np.sqrt(delta_x_m**2 + delta_y_m**2)
                    # print('delta_x_m =',delta_x_m)
                    # print('delta_y_m =',delta_y_m)
                    # print('distance_from_center_m =',distance_from_center_m)
                    if draw:
                        drawframe = self.draw_center(frame, center)
                        drawframe = self.draw_center(frame,(int(frame_center[0]),int(frame_center[1])))
                        #Save the image
                        text = str(tvecs)
                        text2 = str(center)
                        cv.aruco.drawDetectedMarkers(frame, corners)
                        cv.drawFrameAxes(frame, self.mtx, self.dist, np.array(rvecs), np.array(tvecs), 0.1)
                        cv.putText(drawframe,text,(50,50),fontFace=cv.FONT_HERSHEY_SIMPLEX,fontScale=1,color=(0,0,0),thickness=2)
                        cv.putText(drawframe,text2,(50,300),fontFace=cv.FONT_HERSHEY_SIMPLEX,fontScale=1,color=(0,0,0),thickness=2)
                        cv.imwrite('draw/'+str(time.time())+'.jpg',drawframe)
                    break

        if not detected:
            if draw:
               cv.imwrite('notdetected/'+str(time.time())+'.jpg',frame)
            rvecs = None
            tvecs = None
            delta_x_m, delta_y_m, distance_from_center_m = None, None, None

        return detected, rvecs, tvecs, delta_x_m, delta_y_m, distance_from_center_m

def vision():
    '''
    Main function of the vision module
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--calibrate', action='store_true', help='Calibrate the camera')
    parser.add_argument('-g', '--generate', action='store_true', help='Generate a ArUco tag')
    parser.add_argument('-d','--detect',action='store_true',help='Detected a ArUco tag')
    args = parser.parse_args()
    data = config.load('config.json')
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    detector = Detector(dictionary,data['aruco']['ID'],data['aruco']['size'],data['aruco']['real_size'])
    if args.calibrate:
        print('Calibration')
        detector.calibration()
    if args.generate:
        detector.generate_markers()

    if args.detect:
        detector.load_calibration()
        picam = Picamera2()
        picam.set_controls({
        "AfMode":controls.AfModeEnum.Continuous,
        "ExposureTime":400,

        })
        picam.start()
        time.sleep(1)
        last_time = time.time()
        # fps = data['fps']
        fps = 100
        while True:
            if time.time() - last_time < 1/fps:
                continue
            picam.capture_file('cam.jpg')
            frame = cv.imread('cam.jpg')
            corners, ids, _ = detector.detect_markers(frame)
            center = None
            drawframe = None
            detected = False
            if ids is not None:
                for i in range(len(ids)):
                    if ids[i] == detector.aruco_id: 
                        detected = True
                        center = detector.get_marker_center(corners[i])
                        drawframe = detector.draw_center(frame, center)
                        #Save the image
                        cv.imwrite('draw.jpg',drawframe)
                        rvecs, tvecs = detector.marker_position(corners[i] )
                        print('x=',tvecs[0][0],'y=',tvecs[0][1],'z=',tvecs[0][1])
                        break
            if not detected:
                rvecs = None
                tvecs = None
            last_time = time.time()

            
        
            

if __name__ == "__main__":
    vision()
