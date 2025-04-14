# General
STATE = {
    "STOP": 0,
    "FORWARD": 1,
    "BACKWARD": 2,
    "TRACKING": 3
}
STATE_LOOKUP = {v: k for k, v in STATE.items()}
DT = 0.1                       # execution period in seconds

# Motor
MIN_VOLTAGE = 12.5             # in volts
MAX_VOLTAGE = 17.0             # in volts
MANUAL_MOTOR_SPEED = 6         # in m/s (with 4S battery, max theoretical speed: 81 turns/sec = 12.7 m/s, max practical speed: 60 turns/sec = 9.4 m/s)
MOTOR_ACCELERATION = 40.0      # in turns/sec^2
STOP_SPEED_THRESHOLD = 0.01    # in turns/sec
PULLEY_RADIUS = 0.025          # in meters (tacking into acount cable radius)
ZIPLINE_LENGTH = 50            # in meters (length of the zipline)

# Ultrasonic sensors
PIN_FRONT = 5                  # GPIO5 (PIN 29)
PIN_BACK = 16                  # GPIO16 (PIN 36)
TIMEOUT1 = 1000                # in microseconds
TIMEOUT2 = 10000               # in microseconds
MAX_DIST = 6.0                 # maximum detection distance, in meters
OBST_THRESHOLD = 0.2           # distance under which an object is considered an obstacle, in meters
NB_READINGS = 1                # number of readings to consider an object as an obstacle

# LEDs
NUM_LEDS = 3                   # number of LEDs
RED = (255, 0, 0)              # RGB colors for red
BLUE = (0, 0, 255)             # RGB colors for blue    
GREEN = (0, 255, 0)            # RGB colors for green
YELLOW = (255, 255, 0)         # RGB colors for yellow

# RC communication
STEERING_PIN = 24              # GPIO24 (PIN 18)
TROTTLE_PIN = 25               # GPIO25 (PIN 22)

PWM_MIN_PULSE_WIDTH = 1000     # in µs
PWM_DEFAULT_PULSE_WIDTH = 1500 # in µs
PWM_MAX_PULSE_WIDTH = 2000     # in µs
GO_STOP_TRHESHOLD = 30         # in µs

REMOTE_COMMAND = {
    "GO_STOP": 0,
    "GO_BACKWARD": 1,
    "GO_FORWARD": 2,
    "GO_TRACKING": 3
}
COMMAND_LOOKUP = {v: k for k, v in REMOTE_COMMAND.items()}

# Camera
FRAME_RATE = 60              # in frames/sec

