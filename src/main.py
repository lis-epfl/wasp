import time
from multiprocessing import Process, Value
from collections import deque
import gpiod

import config
import rf_receiver_ctrl
import state_machine
import leds_ctrl
import ultrasonic_ctrl
#import motor_control


def remote_control(shared_val):
    '''
    Constantly check for button press through RF receiver (CC1101).
    '''
    rf_receiver_ctrl.ini_rf_reciever() # Initialize CC1101

    # Local variables
    last_tick_ns = None
    last_level = None
    buffer = deque(maxlen=config.SEQUENCE_SIZE)
    last_tick_ns = None
    last_event_type = None
    bit_count = 0
    waiting_for_rising_edge = False
    detected_sequence_id = 0

    chip = gpiod.Chip('gpiochip0')
    line = chip.get_line(config.GDO2_PIN)
    line.request(consumer='sequence_detector', type=gpiod.LINE_REQ_EV_BOTH_EDGES)

    last_edge_time_ns = time.time_ns()

    try:
        while True:
            event = line.event_wait(sec=1)
            current_time_ns = time.time_ns()
            
            # Check for idle timeout
            if current_time_ns - last_edge_time_ns > config.TIMEOUT_THRESHOLD_US * 1000:
                buffer.clear()
                last_tick_ns = None
                last_event_type = None
                last_level = None
                waiting_for_rising_edge = True
                #ini_rf_reciever() # Reset CC1101
                #print("[Timeout] Buffer reset!")
            
            if event:
                evt = line.event_read()
                last_edge_time_ns = time.time_ns()
                
                # Ignore until first rising edge after timeout
                if waiting_for_rising_edge:
                    if evt.type == gpiod.LineEvent.RISING_EDGE:
                        #print("[Sync] Detected rising edge, starting fresh")
                        waiting_for_rising_edge = False
                        last_tick_ns = evt.sec * 1_000_000_000 + evt.nsec
                        last_event_type = evt.type
                        last_level = 1
                    continue  # Skip further processing until resynchronized
                
                # Normal processing after sync
                level = 1 if evt.type == gpiod.LineEvent.RISING_EDGE else 0
                current_tick_ns = evt.sec * 1_000_000_000 + evt.nsec

                # Check edge type change
                if last_tick_ns is not None and evt.type != last_event_type:
                    delta_ns = current_tick_ns - last_tick_ns
                    delta_us = delta_ns / 1000
                    bit_count = round(delta_us / config.BIT_DURATION)

                    if bit_count > 0:
                        bits_to_add = str(last_level) * bit_count
                        buffer.extend(bits_to_add)
                        #print(bits_to_add, end='', flush=True) # Print all bits detected

                # Update last tick, level, event type
                last_tick_ns = current_tick_ns
                last_level = level
                last_event_type = evt.type

                # Sequence detection
                if len(buffer) >= config.SEQUENCE_SIZE:
                    current_seq = ''.join(buffer)
                    for idx, seq in enumerate(config.BUTTON_SEQUENCES, 1):
                        if current_seq == seq:
                            detected_sequence_id = idx
                            #print(f"\nButton {detected_sequence_id} pushed!")
        
            # Update shared value between processes
            with shared_val.get_lock():
                shared_val.value = detected_sequence_id

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        line.release()
        chip.close()


def entire_system(shared_val):
    '''
    Main function to run the entire system.
    '''
    # Initialize peripherals
    leds = leds_ctrl.leds_init()
    #ultrasonic_ctrl.ultrasonic_init()

    # Initial state
    state = config.STATE["STOP"]

    while True:
        # Get remote control command
        with shared_val.get_lock():
            remote_command = config.COMMAND_LOOKUP.get(shared_val.value, "NONE") # default value is "NONE"

        # Get distance sensors readings

           

        time.sleep(0.01) 


         print(f"Received Command: {remote_command}")

        #obst = is_there_obstacle() # True if obstacle detected, False otherwise


        # Update state
        state = state_machine.update(state, remote_command, obst)
        
        # Display current state
        leds_ctrl.leds_set_color(leds, state)

  

        # # Replace this with proper function that returns either true or false
        # front_value = ultrasonic_ctrl.get_distance(config.PIN_FRONT)
        # back_value = ultrasonic_ctrl.get_distance(config.PIN_BACK)

        # print('Back: {:.2f} m    |    Front: {:.2f} m'.format(front_value, back_value))

        # #target_velocity = get_target_velocity() # Get UAV linear velocity from camera

        # #set_cart_velocity(state, target_velocity)

        # time.sleep(config.DT)


if __name__ == "__main__":
    shared_val = Value('i', 0)  # Shared 'i' = integer (default value 0) between processes

    p1 = Process(target=remote_control, args=(shared_val,))
    p2 = Process(target=entire_system, args=(shared_val,))

    p1.start()
    p2.start()

    p1.join()  # Runs indefinitely, stop manually if needed
