import board
import neopixel

import config

def leds_init():
    """
    Initialize the NeoPixel leds
    :return: NeoPixel object
    """
    leds = neopixel.NeoPixel(board.D12, config.NUM_LEDS, brightness=1.0, auto_write=False)
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
