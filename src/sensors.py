import time
import leds 
        

def end_of_line(distance, threshold, motor, led_front, led_back):
        if distance < threshold:
            print("###########################################")
            print(distance)
            motor.stop()
            # led_front.set_color(leds.ORANGE)
            # led_back.set_color(leds.ORANGE)
            # led_back.set_brightness(1)
            # led_front.set_brightness(1)
            return False
        else:
            return True      