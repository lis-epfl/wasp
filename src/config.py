import cv2 as cv

# General
STATE = {
    'STOP': 0,
    'FORWARD': 1,
    'BACKWARD': 2,
    'TRACKING': 3
}
STATE_LOOKUP = {v: k for k, v in STATE.items()}
DT = 0.12                      # execution period in seconds
DT_VISION = 0.10               # execution period for vision in seconds
CALIBRATING = False            # set to True to calibrate the line distance


# Motor
NB_CELLS = 6                                          # Number of cells in the battery
MIN_VOLTAGE = NB_CELLS*3.3                            # Minimum safe voltage (discharged), in V
MAX_VOLTAGE = NB_CELLS*4.2                            # Maximum voltage (fully charged), in V
SOFT_MAX_CURRENT = 40.0                               # in A
HARD_MAX_CURRENT = 60.0                               # in A
SPEED_CONSTANT = 330                                  # in RMP/V
TORQUE_CONSTANT = 0.025                               # in Nm/A
LOSS_CONSTANT = 0.68                                  # to account for voltage drops and control overhead
SOFT_MAX_TORQUE = SOFT_MAX_CURRENT * TORQUE_CONSTANT  # in Nm
HARD_MAX_TORQUE = HARD_MAX_CURRENT * TORQUE_CONSTANT  # in Nm

POS_GAIN = 20.0                # Proportional gain for position loop [(rev/s) / rev]
VEL_GAIN = 0.10                # Proportional gain for velocity loop  [Nm / (rev/s)]
INTEGRATOR_GAIN = 0.33         # Integral gain for velocity loop [Nm / (rev/s^2)]
BANG_BANG_GAIN = 10            # Gain for the semi-position control when using manual mode (replicate bang–bang controller)
MAX_TRACKING_SPEED = 14        # in m/s (max speed for tracking)
MAX_ACCELERATION = 30.0        # in turns/sec^2
STOP_SPEED_THRESHOLD = 0.01    # in turns/sec
PULLEY_RADIUS = 0.025          # in meters (tacking into acount cable radius)

MAX_MANUAL_SPEED_CALIB = 1.5     # in m/s (for calibration purposes)
INITAL_MOTOR_POS_CALIB = 1000    # in meters
ZIPLINE_START_CALIB = 500        # in meters
ZIPLINE_LENGTH_CALIB = 1500      # in meters

MAX_MANUAL_SPEED = 14            # in m/s (with 4S battery, max theoretical speed: 81 turns/sec = 12.7 m/s, max practical speed: 60 turns/sec = 9.4 m/s)
INITAL_MOTOR_POS = 0.0           # in meters
ZIPLINE_START = 0.0              # in meters
ZIPLINE_LENGTH = 70              # in meters


# Ultrasonic sensors
PIN_FRONT = 5                  # GPIO5 (PIN 29)
PIN_BACK = 16                  # GPIO16 (PIN 36)
TIMEOUT1 = 1000                # in microseconds
TIMEOUT2 = 10000               # in microseconds
MAX_DIST = 6.0                 # maximum detection distance, in meters
OBST_THRESHOLD = 0.1           # distance under which an object is considered an obstacle, in meters


# LEDs
NUM_LEDS = 3                   # number of LEDs
RED = (255, 0, 0)              # RGB colors for red
BLUE = (0, 0, 255)             # RGB colors for blue    
GREEN = (0, 255, 0)            # RGB colors for green
YELLOW = (255, 255, 0)         # RGB colors for yellow


# RC communication
BUTTON_PIN = 24                # GPIO24 (PIN 18)
TROTTLE_PIN = 25               # GPIO25 (PIN 22)
PWM_MIN_PULSE_WIDTH = 1000     # in µs
PWM_DEFAULT_PULSE_WIDTH = 1500 # in µs
PWM_MAX_PULSE_WIDTH = 2000     # in µs
GO_STOP_THRESHOLD = 30         # in µs
STAY_TRACKING_THRESHOLD = 75   # in µs
BUTTON_TOGGLE_THRESHOLD = 200  # in µs
REMOTE_COMMAND = {
    'GO_STOP': 0,
    'GO_BACKWARD': 1,
    'GO_FORWARD': 2,
    'GO_TRACKING': 3
}
COMMAND_LOOKUP = {v: k for k, v in REMOTE_COMMAND.items()}


# Camera
CAM_HEIGHT = 4608                               # camera default resolution (maximum), in pixels
CAM_WIDTH = 2592                                # camera default resolution (maximum), in pixels
RES_DROP = 5                                    # resolution drop factor
CAM_HEIGHT_LOW = int(CAM_HEIGHT/RES_DROP)       # lower resolution settings, in pixels
CAM_WIDTH_LOW = int((CAM_HEIGHT_LOW / 16) * 9)  # lower resolution settings, in pixels
FRAME_RATE = 60                                 # in frames/sec
CALIBRATION_SQUARE = 0.0323                     # size of the squares in the checkerboard, in meters
CHECKERBOARD_SHAPE = (4, 7)                     # number of inner corners per row and column !!nb of squares - 1!!
NB_IMAGES_CALIBRATION = 20                      # number of images to capture for calibration
ARUCO_DICT = cv.aruco.DICT_5X5_250              # 5x5 dictionary with 250 unique markers
ARUCO_ID = 77                                   # exact marker ID to be detected between 0 and 249
ARUCO_PIXEL_SIZE = 400                          # size of the ArUco marker, in pixels (for ArUo generation)
ARUCO_REAL_SIZE = 0.1355                        # size of the ArUco marker, in meters (to be measured in real life)
EXPOSURE_TIME = 1500                            # in microseconds
ANALOGUE_GAIN = 20.0                            # in dB


# Tracking
MAX_CNT_MOVING_BLINDLY = 3     # number of iterations applying last detected position without any new detection


# Wind sensor
SERIAL_PORT_LI550= '/dev/ttyUSB0'
BAUD_RATE_LI550 = 115200             # in bps
WIND_AXIS_LENGTH = 10                # in meters/s
NUM_PAST_VECTORS = 10                # number of past vector displayed
INIT_TIME_LI550 = 10                 # time to wait for the LI550 to be ready, in seconds
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
    'Tracking error [m]',
    'Voltage [V]',
    'Current [A]',
    'x_ref [m]',
] + list(LI550_MAPPING.values())
