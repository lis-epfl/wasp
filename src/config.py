# File use to config the different file of the project and for load the data from a json file
import sys
import os
import json

def default():
    '''Create a new config file with the default values'''
    data = {
    "fps": 50,
    "aruco":{
        "dictioénary": "DICT_6X6_250",
        "ID": 23,
        "size": 500,
        "real_size": 0.169,
        "draw": 0,
        "calibration_square":0.032
    },
    "leds": {
        "led_front": {
            "pin": "D24" ,
            "num_leds": 1
        },
        "led_back": {
            "pin": "D6",
            "num_leds": 1
        },
        "led_bottom": {
            "pin": "D10",
            "num_leds": 1
        }
    },
    "motor": {
        "channels": 8,
        "channel": 0,
        "K_POS": 0.5,
        "K_SPEED": 0.5,
        "departure":0.35
    },
    "ultrasonics": {
        "bw-threshold":60,
        "fw-threshold":100,
        "front": {
            "pin": 22
        },
        "back": {
            "pin": 26

        }
    }

    }
    # Write the data to the config file
    with open('config.json', 'w') as file:
        json.dump(data, file, indent=4)

def load(file_name):
    '''Load the data from the config file'''
    with open(file_name, 'r') as file:
        data = json.load(file)
    return data 

def get_led_pins(data,led):
    '''Get the pins of the led from the data'''
    return data['leds'][led]['pin']
    

if __name__ == '__main__':
    default()
    sys.exit(0)