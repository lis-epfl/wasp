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
MIN_VOLTAGE = 12.0             # in volts
MAX_VOLTAGE = 17.0             # in volts
MANUAL_MOTOR_SPEED = 20        # in turns/sec (with 4S battery, max theoretical speed: 81 turns/sec, max practical speed: 60 turns/sec)
MOTOR_ACCELERATION = 40.0      # in turns/sec^2
STOP_SPEED_THRESHOLD = 0.5     # in turns/sec
PULLEY_RADIUS = 0.025          # in meters (tacking into acount cable radius)
ZIPLINE_LENGTH = 5.0           # in meters (length of the zipline)

# Ultrasonic sensors
PIN_BACK = 5                   # GPIO5 (PIN 29)
PIN_FRONT = 16                 # GPIO16 (PIN 36)
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


# RF communication
CC1101_CONFIG = {
    0x00: 0x0D,                 # IOCFG2        GDO2 Output Pin Configuration
    0x02: 0x0D,                 # IOCFG0        GDO0 Output Pin Configuration
    0x03: 0x47,                 # FIFOTHR       RX FIFO and TX FIFO Thresholds
    0x08: 0x32,                 # PKTCTRL0      Packet Automation Control
    0x0B: 0x06,                 # FSCTRL1       Frequency Synthesizer Control
    0x0D: 0x10,                 # FREQ2         Frequency Control Word, High Byte
    0x0E: 0xB0,                 # FREQ1         Frequency Control Word, Middle Byte
    0x0F: 0x71,                 # FREQ0         Frequency Control Word, Low Byte
    0x10: 0x76,                 # MDMCFG4       Modem Config: BW ≈ 58 kHz, DRATE_E = 6
    0x11: 0x83,                 # MDMCFG3       DRATE_M = 131 (for 2.4 kbps)
    0x12: 0x30,                 # MDMCFG2       Modem Configuration
    0x13: 0x20,                 # MDMCFG1       Modem Configuration
    0x14: 0xF7,                 # MDMCFG0       Modem Configuration
    0x15: 0x15,                 # DEVIATN       Modem Deviation Setting
    0x18: 0x18,                 # MCSM0         Main Radio Control State Machine Configuration
    0x19: 0x16,                 # FOCCFG        Frequency Offset Compensation Configuration
    0x1B: 0xFB,                 # WORCTRL       Wake On Radio Control
    0x22: 0x11,                 # FREND0        Front End TX Configuration
    0x23: 0xE9,                 # FSCAL3        Frequency Synthesizer Calibration
    0x24: 0x2A,                 # FSCAL2        Frequency Synthesizer Calibration
    0x25: 0x00,                 # FSCAL1        Frequency Synthesizer Calibration
    0x26: 0x1F,                 # FSCAL0        Frequency Synthesizer Calibration
    0x2C: 0x81,                 # TEST2         Various Test Settings
    0x2D: 0x35,                 # TEST1         Various Test Settings
    0x2E: 0x09,                 # TEST0         Various Test Settings
}
GDO2_PIN = 25                   # GPIO25 (PIN 22)
BIT_DURATION = 416.7            # duration of each bit at its state in microseconds
SEQUENCE_SIZE = 121             # sequence length in bits
TIMEOUT_THRESHOLD_US = 30000    # 30 ms 
BUTTON_SEQUENCES = [
    "1011001011001011011011011011011011011011011011011011011011011011011011011001001001011001011001011001011001011001011001011",
    "1011001011001011011011011011011011011011011011011011011011011011011011011001011001001001011001011001011001011001011001011",
    "1011001011001011011011011011011011011011011011011011011011011011011011011001011001011001001001011001011001011001011001011",
    "1011001011001011011011011011011011011011011011011011011011011011011011011001011001011001011001001001011001011001011001011",
    "1011001011001011011011011011011011011011011011011011011011011011011011011001011001011001011001011001001001011001011001011"
]
REMOTE_COMMAND = {
    "NONE": 0,
    "GO_TRACKING": 1,
    "GO_BACKWARD": 2,
    "NOTHING": 3,
    "GO_FORWARD": 4,
    "GO_STOP": 5
}
COMMAND_LOOKUP = {v: k for k, v in REMOTE_COMMAND.items()}


# Camera
FRAME_RATE = 60              # in frames/sec

