import neopixel
import time
import board
GREEN = (0, 255, 0)
RED = (255, 0, 0)
ORANGE = (255, 165, 0)
WHITE = (255, 255, 255)

class Led:
    def __init__(self,pin, num_leds):
        pin = getattr(board, pin)
        self.led = neopixel.NeoPixel(pin, num_leds, brightness=0.2, auto_write=False, pixel_order=neopixel.RGB)
    def set_color(self, color):
        self.led.fill(color)
        self.led.show()

    def set_led(self, index, color):
        self.led[index] = color
        self.led.show()

    def clear(self):
        self.led.fill((0, 0, 0))
        self.led.show()

    def set_brightness(self, brightness):
        self.led.brightness = brightness
        self.led.show()



def starting_up(leds):
    for led in leds:
        led.set_color(WHITE)
        led.set_brightness(1)
        time.sleep(0.5)
        led.clear()
        time.sleep(0.5)
