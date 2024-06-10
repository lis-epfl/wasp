# File use to config the different file of the project and for load the data from a json file
import sys
import os
import json

def default():
    '''Create a new config file with the default values'''
    data = {
        "fps": 30,
        "aruco":{
        "dictionary": "DICT_6X6_250",
        "ID": 23,
        "size": 200,
        "real_size": 0.1,
        "draw": 1
    },
        'leds': {
            'led_front': {
                'pin': 'D4',
                'num_leds': 1
            },
            'led_back': {
                'pin': 'D5',
                'num_leds': 1
            },
            'led_bottom': {
                'pin': 'D6',
                'num_leds': 1
            }
        },
        'motor': {
            'channels': 8,
            'channel': 0,
            'K_POS': 0.5,
            'K_SPEED': 0.5
        },
        "ultrasonics": {
        "threshold":100,
        "front": {
            "pin": "D7"
        },
        "back": {
            "pin": "D8"
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