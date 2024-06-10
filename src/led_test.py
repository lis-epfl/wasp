from gpiozero import DigitalOutputDevice
from time import sleep

# Define the GPIO pin to use
LED_PIN = 24

# Setup GPIO
led = DigitalOutputDevice(LED_PIN)

def send_bit(bit):
    if bit:
        # Send a 1 bit
        led.on()
        sleep(0.0000008)  # 0.8µs
        led.off()
        sleep(0.00000045)  # 0.45µs
    else:
        # Send a 0 bit
        led.on()
        sleep(0.0000004)  # 0.4µs
        led.off()
        sleep(0.00000085)  # 0.85µs

def send_byte(byte):
    for i in range(8):
        print('Sending bit:', (byte >> (7 - i)) & 1)
        send_bit((byte >> (7 - i)) & 1)

def send_color(red, green, blue):
    send_byte(green)
    send_byte(red)
    send_byte(blue)

def reset():
    led.off()
    sleep(0.00008)  # 80µs to reset

def set_colors(colors):
    # Clear any prior state
    reset()
    # Send the color data for each LED
    for color in colors:
        send_color(*color)
    # Reset again to latch the color
    reset()

# Define colors for the 8 LEDs (use (red, green, blue) tuples)

try:
    while True:
        

finally:
    # Clean up GPIO
    led.close()
