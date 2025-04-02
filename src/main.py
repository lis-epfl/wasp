import time
from multiprocessing import Process, Value
from collections import deque
import gpiod

import config
import rf_receiver_ctrl
import state_machine
import leds_ctrl
import ultrasonic_ctrl
import motor_ctrl


def remote_control(shared_val):
    '''
    Constantly check for button press through RF receiver (CC1101).
    '''
    rf_receiver_ctrl.ini_rf_reciever() # Initialize CC1101

    # Necessary local variables
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
                        # else:
                        #     detected_sequence_id = 0
        
            # Update shared value between processes
            with shared_val.get_lock():
                shared_val.value = detected_sequence_id

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        line.release()
        chip.close()


def main(shared_val):
    # Initialize peripherals
    leds = leds_ctrl.leds_init()
    ultrasonic_ctrl.ultrasonic_init()
    odrv = motor_ctrl.motor_init()

    # Time variables
    time_start = 0
    time_end = 0

    # Initial state
    state = config.STATE["STOP"]
    last_state = state

    # Deques to store the last ultrasonic sensor readings
    front_readings = deque(maxlen=config.NB_READINGS)
    back_readings = deque(maxlen=config.NB_READINGS)

    want_to_stop = False

    while True:
        time_start = time.time()

        if want_to_stop:
            if odrv.axis0.vel_estimate < config.STOP_SPEED_THRESHOLD:
                # Motor is stopped
                want_to_stop = False

        else:
            # Get remote control command
            with shared_val.get_lock():
                remote_command = config.REMOTE_COMMAND.get(config.COMMAND_LOOKUP.get(shared_val.value, "NONE"), 0)

            # Get distance sensor readings
            front_value = ultrasonic_ctrl.get_distance(config.PIN_FRONT)
            back_value = ultrasonic_ctrl.get_distance(config.PIN_BACK)
            print('Back: {:.2f} m    |    Front: {:.2f} m'.format(front_value, back_value))

            # # Filter valid distance sensor values
            # if front_value is not None and front_value <= config.MAX_DIST:
            #     front_readings.append(front_value)
            # if back_value is not None and back_value <= config.MAX_DIST:
            #     back_readings.append(back_value)

            # # Determine obstacle presence
            # obstacle_forward = len(front_readings) == config.NB_READINGS and all(d <= config.OBST_THRESHOLD for d in front_readings)
            # obstacle_backward = len(back_readings) == config.NB_READINGS and all(d <= config.OBST_THRESHOLD for d in back_readings)

            obstacle_forward = False
            obstacle_backward = False

            # # Update state
            state = state_machine.update(last_state, remote_command, obstacle_forward, obstacle_backward)
            last_state = state

            if state == config.STATE["STOP"]:
                # Stop the motor
                want_to_stop = True

        # Print current state
        log_message = f"Last command: {config.COMMAND_LOOKUP.get(remote_command, 'UNKNOWN')}  |   " \
                      f"Obstacle forward: {obstacle_forward}   |   Obstacle backward: {obstacle_backward}   |   " \
                      f"Current state: {config.STATE_LOOKUP.get(state, 'UNKNOWN')}"
        #print(log_message)

        # Display current state with LEDs
        leds_ctrl.leds_set_color(leds, state)

        # Set motor velocity based on state
        target_velocity = config.MOTOR_SPEED
        # print(motor_ctrl.compute_linear_speed(odrv.axis0.vel_estimate))
        motor_ctrl.set_cart_velocity(odrv, state, target_velocity)

        # Sleep to respect the desired loop time
        time_end = time.time()
        if time_end - time_start < config.DT:
            time.sleep(config.DT - (time_end - time_start))
        else:
            print(f"Execution time exceeded {config.DT} seconds.")


if __name__ == "__main__":
    shared_val = Value('i', 0)  # Shared 'i' = integer (default value 0) between processes

    p1 = Process(target=remote_control, args=(shared_val,))
    p2 = Process(target=main, args=(shared_val,))

    p1.start() # Start RF receiver process
    p2.start() # Start entire system process

    p1.join()