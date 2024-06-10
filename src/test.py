import time 
from adafruit_servokit import ServoKit
from adafruit_bno055 import BNO055_I2C
import board
import busio
# # # Set channels on the FeatherWing
kit = ServoKit(channels=8)

# Assuming the ESC is connected to channel 0
ESC_CHANNEL = 0
#Neutral position of the ESC
kit.continuous_servo[ESC_CHANNEL].throttle = 0.0
input("Press Enter to continue...")
kit.continuous_servo[ESC_CHANNEL].throttle = 1
input("Press Enter to continue...")
kit.continuous_servo[ESC_CHANNEL].throttle = 0.0
input("Press Enter to continue...")
#Full speed reverse








# #Read the IMU

# i2c = busio.I2C(board.SCL, board.SDA)

# bno = BNO055_I2C(i2c)

# while True:
#     print(bno.euler)
#     time.sleep(0.1)

