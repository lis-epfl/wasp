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


def leds_set_color(leds, state, tracking_error, leds_off_before, in_calibration_mode):
    """
    Set the color of the LEDs based on the state
    :param leds: NeoPixel object
    :param state: Current state of the cart
    """
    if state == config.STATE["STOP"]:
        leds.fill(config.RED)
        leds.show()

    elif (state == config.STATE["FORWARD"]) or (state == config.STATE["BACKWARD"]):
        if in_calibration_mode:
            if leds_off_before:
                leds.fill(config.BLUE)
            else:
                leds.fill((0, 0, 0))
            leds_off_before = not leds_off_before  # Toggle the state for the next call
        else:
            leds.fill(config.BLUE)
        leds.show()

    elif state == config.STATE["TRACKING"]:
        if tracking_error is None:
            # no finding the ArUco tag
            if leds_off_before:
                leds.fill(config.GREEN)
            else:
                leds.fill((0, 0, 0))
            leds_off_before = not leds_off_before  # Toggle the state for the next call
        else:
            # tracking the ArUco tag
            leds.fill(config.GREEN)
        leds.show()

    else:
        leds.fill((0, 0, 0)) # Off
        leds.show()

    return leds_off_before


def leds_set_color_calibration(leds, leds_off_before):
    """
    Set the color of the LEDs in calibration mode
    :param leds: NeoPixel object
    :param leds_off_before: Boolean indicating if the LEDs were off before
    """
    if leds_off_before:
        leds.fill(config.PURPLE)
    else:
        leds.fill((0, 0, 0))
    leds_off_before = not leds_off_before  # Toggle the state for the next call
    leds.show()

    return leds_off_before


def leds_show_setpoint_calibration(leds):
    """
    Set the color of the LEDs to show the setpoint in calibration mode
    :param leds: NeoPixel object
    """
    leds.fill(config.PURPLE)
    leds.show()


def leds_error_warning(leds, leds_off_before):
    """
    Set the color of the LEDs in error mode
    :param leds: NeoPixel object
    :param leds_off_before: Boolean indicating if the LEDs were off before
    """
    if leds_off_before:
        leds.fill(config.YELLOW)
    else:
        leds.fill((0, 0, 0))    
    leds.show()
    leds_off_before = not leds_off_before  # Toggle the state for the next call

    return leds_off_before
