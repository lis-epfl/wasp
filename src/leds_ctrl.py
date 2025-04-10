import board
import neopixel

import config

def leds_init():
    """
    Initialize the NeoPixel leds
    :return: NeoPixel object
    """
    leds = neopixel.NeoPixel(board.D12, config.NUM_LEDS, brightness=1.0, auto_write=False) # GPIO12 (PIN 32) 
    return leds

def leds_set_color(leds, state):
    """
    Set the color of the LEDs based on the state
    :param leds: NeoPixel object
    :param state: Current state of the cart
    """
    if state == config.STATE["STOP"]:
        leds.fill(config.RED)
        leds.show()

    elif (state == config.STATE["FORWARD"]) or (state == config.STATE["BACKWARD"]):
        leds.fill(config.BLUE) 
        leds.show()

    elif state == config.STATE["TRACKING"]:
        leds.fill(config.GREEN)
        leds.show()

    else:
        leds.fill((0, 0, 0)) # Off
        leds.show()

def leds_error_warning(leds, leds_off_before):
    """
    Toggle the color of the LEDs between yellow and off
    :param leds: NeoPixel object
    """
    if leds_off_before:
        leds.fill(config.YELLOW)
        print("on")
    else:
        leds.fill((0, 0, 0))
        print("off")
    
    leds.show()
    leds_off_before = not leds_off_before  # Toggle the state for the next call

    return leds_off_before
