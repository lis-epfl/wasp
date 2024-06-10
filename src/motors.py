import board
import busio
from adafruit_servokit import ServoKit
import simple_pid as pid
import numpy as np
import time


K_P = 1 #0.5 #0.25
K_I = 0.5
K_D = 0.5
# Output between 0 and 1
PID = pid.PID(K_P,K_I,K_D,setpoint=0,output_limits=(-0.2,0.6))
LOW_PASS = 1000

class Motor:
    def __init__(self,i2c,channels=8,channel=0):
        self.kit = ServoKit(channels=channels)
        self.channel = channel
        self.i2c = i2c
    def set_speed(self,speed):
        self.kit.continuous_servo[self.channel].throttle = speed
        self.i2c.unlock()
    def stop(self):
        self.kit.continuous_servo[self.channel].throttle = 0.0
        self.i2c.unlock()


def speed_controller(real_pos,imu_angle,distance,prev_speed,dt):
    #print("Angle = ",imu_angle)
    # print("Real Pos = ", real_pos)
    imu_angle = np.deg2rad(imu_angle)
    #corrected_pos = real_pos - np.tan(imu_angle) * distance
    corrected_pos = -real_pos
    #print("corrected:",corrected_pos)
    #print("Corrected Pos = ",corrected_pos
    output = PID(corrected_pos)
    #print("OUTPUT: ",output)
    lp = low_pass(output,prev_speed,LOW_PASS,dt)
    return lp + 0.2

def low_pass(curr, prev, lowpass_cutoff, dt):
    a = curr-prev
    output = prev + a*(1.0 - np.e**(-dt*lowpass_cutoff))
    return output
    



    


    