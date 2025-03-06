import board
import neopixel
import time

# Set up the NeoPixel
pixel_pin = board.D12  # GPIO18
num_pixels = 3  # Change to match the number of NeoPixels

pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=1.0, auto_write=False)

def color_wipe(color, wait):
    pixels.fill(color)
    pixels.show()
    time.sleep(wait)

while True:
    color_wipe((255, 0, 0), 1)  # Red
    color_wipe((0, 255, 0), 1)  # Green
    color_wipe((0, 0, 255), 1)  # Blue
