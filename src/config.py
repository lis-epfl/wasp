import cv2 as cv

# General
STATE = {
    'STOP': 0,
    'FORWARD': 1,
    'BACKWARD': 2,
    'TRACKING': 3,
    'TAKE_OFF': 4
}
STATE_LOOKUP = {v: k for k, v in STATE.items()}
DT_MAIN = 0.1   # execution period in seconds (0.04 without the wind sensor, 0.12 with it) (set to 0.1 for save_images = 0, 0.2 for save_images = 1 or 2)
DT_WIND = 0.1   # execution period for vision in seconds (7Hz) (can be set to 0.06666666 if save_images is False)
DT_RC = 0.01
ENABLE_WIND_SENSING = True

# Battery
NB_CELLS = 8                                            # Number of cells in the battery
MIN_VOLTAGE = NB_CELLS*3.3                              # Minimum safe voltage (discharged), in V
MAX_VOLTAGE = NB_CELLS*4.2                              # Maximum voltage (fully charged), in V


# Motor
SOFT_MAX_CURRENT = 30.0                                 # in A
HARD_MAX_CURRENT = 60.0                                 # in A
SPEED_CONSTANT = 330                                    # in RMP/V
TORQUE_CONSTANT = 0.025                                 # in Nm/A
SOFT_MAX_TORQUE = SOFT_MAX_CURRENT * TORQUE_CONSTANT    # in Nm
HARD_MAX_TORQUE = HARD_MAX_CURRENT * TORQUE_CONSTANT    # in Nm
PULLEY_RADIUS = 0.02025                                 # in meters


# Controller
POS_GAIN = 20.0                                         # Proportional gain for position loop [(rev/s) / rev]
VEL_GAIN = 0.10                                         # Proportional gain for velocity loop  [Nm / (rev/s)]
INTEGRATOR_GAIN = 0.07                                  # Integral gain for velocity loop [Nm / (rev/s^2)]
ACCELERATION = 8.0                                      # in m/s^2 (10 works with fully chareged 6S)
DECELERATION = 4.0                                      # in m/s^2 (take ~20 m to decelerate from 14 m/s so the line must be at least 50)
DECELERATION_MARGIN = 1.2                               # Gain to enable the controller to compute the deceleration curve
STOP_SPEED_THRESHOLD = 0.01                             # in m/s
MAX_SPEED = 14.0                                        # in m/s (with 6S battery, max 14 m/s)
MAX_SPEED_CALIB = 2.0                                   # in m/s (for calibration purposes)
INITIAL_MOTOR_POS_CALIB = 1000                          # in meters
ZIPLINE_START_CALIB = 500                               # in meters
ZIPLINE_LENGTH_CALIB = 1500                             # in meters


# Tracking
P_GAIN_VISION = 1.0
D_GAIN_VISION = 0.5
MAX_CNT_MOVING_BLINDLY = 1/DT_MAIN                              # number of iterations applying last detected position without any new detection
CUT_OFF_FREQUENCY_TRACKING = 5                          # in Hz
CATCH_UP_GAIN = 1.0                                     # 3 working well with acc of 3 and max speed of 11


# LEDs
NUM_LEDS = 2                                            # number of LEDs
RED = (255, 0, 0)                                       # RGB colors for red
BLUE = (0, 0, 255)                                      # RGB colors for blue
GREEN = (0, 255, 0)                                     # RGB colors for green
YELLOW = (255, 255, 0)                                  # RGB colors for yellow
PURPLE = (102, 51, 153)                                 # RGB colors for purple
TURQUOISE = (0, 255, 255)                               # RGB colors for turquoise


# RC communication
STEERING_PIN = 22                                       # GPIO24
TROTTLE_PIN = 23                                        # GPIO23
BUTTON_PIN = 24                                         # GPIO22
SWITCH_PIN = 25                                         # GPIO25
KNOBL_PIN = 26                                          # GPIO26
KNOBR_PIN = 27                                          # GPIO27
PWM_MIN_PULSE_WIDTH = 1050                              # in µs
PWM_DEFAULT_PULSE_WIDTH = 1500                          # in µs
PWM_MAX_PULSE_WIDTH = 1950                              # in µs
GO_STOP_THRESHOLD = 200                                 # in µs
BUTTON_TOGGLE_THRESHOLD = 200                           # in µs
CALIB_SETPOINTS_THRESHOLD = 300                         # in µs
REMOTE_COMMAND = {
    'GO_STOP': 0,
    'GO_BACKWARD': 1,
    'GO_FORWARD': 2,
    'GO_TRACKING': 3,
    'GO_TAKE_OFF': 4
}
COMMAND_LOOKUP = {v: k for k, v in REMOTE_COMMAND.items()}


# Camera
CAM_HEIGHT = 1456                                   # camera default resolution (maximum), in pixels
CAM_WIDTH = 1088                                    # camera default resolution (maximum), in pixels
RES_DROP = 1.0                                      # resolution drop factor
ASPECT_RATIO = CAM_HEIGHT / CAM_WIDTH               # aspect ratio (4/3)
CAM_HEIGHT_LOW = int(CAM_HEIGHT / RES_DROP)         # lower resolution settings, in pixels
CAM_WIDTH_LOW = int(CAM_HEIGHT_LOW / ASPECT_RATIO)  # lower resolution settings, in pixels
FRAME_RATE = 60                                     # in frames/sec
CALIBRATION_SQUARE = 0.0323                         # size of the squares in the checkerboard, in meters
CHECKERBOARD_SHAPE = (4, 7)                         # number of inner corners per row and column !!nb of squares - 1!!
NB_IMAGES_CALIBRATION = 300                         # number of images to capture for calibration
ARUCO_DICT = cv.aruco.DICT_4X4_50                   # 5x5 dictionary with 250 unique markers
ARUCO_ID = 7                                        # exact marker ID to be detected between 0 and 249
ARUCO_PIXEL_SIZE = 400                              # size of the ArUco marker, in pixels (for ArUo generation)
ARUCO_REAL_SIZE = 0.136                             # size of the ArUco marker, in meters (to be measured in real life)
EXPOSURE_TIME = 200                                   # in microseconds (set 29 µs for outside, and 3000 µs for inside)
ANALOGUE_GAIN = 1.0                                 # in dB
SAVE_IMAGES = 1                                     # save images during operation (0, no images saved, 1 save raw images, 2 save process images, 3 save video stream)


# Marker detection
AUTO_EXPOSURE = False                                # Enable auto exposure (used to be False)
RECENTER_ORIGIN = False                             #  If we alter the center of the image (used to be True)
ADVANCED_PARAMETERS = True                          # Enable advanced parameters for ArUco detection (used to be False)
PRE_PROCESS = True                                  # Enable pre-processing of the image (used to be False)
SOLVER = 2                                          # 0: standard solver, 1: PnP solver, 2: PnP solver with initial guess as last pose
CROP_X = 0.0                                        # percentage of image to crop (0.1 = 10%)
CROP_Y = 0.0                                        # percentage of image to crop (0.1 = 10%)
RES_DROP_PRE = 1.0                                  # resolution drop factor for pre-processing (used to be 1.0)
UNDISTORT = True                                    # set False to work in distorted (raw) domain
STARTING_OFFSET = 0.0                               # starting offset applied at the beginning and end of the zipeline, in meters
TAKE_OFF_POSITION_THRESHOLD = 0.2                   # percentage of the zipline length defining the take-off zones at each end of the zipline


# Wind sensor
SERIAL_PORT_LI550= '/dev/ttyUSB0'
BAUD_RATE_LI550 = 115200                            # in bps
WIND_AXIS_LENGTH = 10                               # in meters/s
NUM_PAST_VECTORS = 10                               # number of past vector displayed
INIT_TIME_LI550 = 10                                # time to wait for the LI550 to be ready, in seconds
LI550_MAPPING = {
    'S': 'Wind 3D norm [m/s]',
    'S2': 'Wind 2D norm [m/s]',
    'D': 'Horizontal wind direction [°]',
    'DV': 'Vertical wind direction [°]',
    'U': 'U Vector [m/s]',
    'V': 'V Vector [m/s]',
    'W': 'W Vector [m/s]',
    'T': 'Temperature [°C]',
    'C': 'Speed of sound [m/s]',
    'H': 'Humidity [%]',
    'DP': 'Dew point [°C]',
    'P': 'Pressure [hPa]',
    'AD': 'Air density [kg/cm³]',
    'PI': 'Pitch [°]',
    'RO': 'Roll [°]',
    'MD': 'Heading [°]',
    'TD': 'TrueHead [°]'
}


# Data logging
CSV_COLUMNS = [
    'Timestamp [s]',
    'Angular position [turns]',
    'Angular velocity [turns/s]',
    'Torque [Nm]',
    'Linear position [m]',
    'Linear speed [m/s]',
    'Voltage [V]',
    'Current [A]',
    'Reference x_ref [m]',
    'Estimated plane position [m]',
    'Estimated plane velocity [m/s]',
    'Reference velocity [m/s]',
    'x ArUco [m]',
    'y ArUco [m]',
    'z ArUco [m]',
    'roll ArUco [deg]',
    'pitch ArUco [deg]',
    'yaw ArUco [deg]'
]

CSV_WIND_COLUMNS = [
    'Timestamp [s]',
] + list(LI550_MAPPING.values())
