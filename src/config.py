# General
STATE = {
    "STOP": 0,
    "FORWARD": 1,
    "BACKWARD": 2,
    "TRACKING": 3
}
DT = 0.1                    # execution period in seconds

# Motor
MOTOR_SPEED = 20            # in turns/sec
PULLEY_RADIUS = 0.05        # in meters (tacking into acount cable radius)

# Ultrasonic sensors
TIMEOUT1 = 1000             # in microseconds
TIMEOUT2 = 10000            # in microseconds
MAX_DIST = 6                # in meters
PIN_FRONT = 5               # GPIO pin
PIN_BACK = 16               # GPIO pin

# LEDs
NUM_LEDS = 3                # Number of LEDs
RED = (255, 0, 0)           # RGB colors for red
BLUE = (0, 0, 255)          # RGB colors for blue    
GREEN = (0, 255, 0)         # RGB colors for green

# Camera


# RF communication