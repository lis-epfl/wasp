import csv
from collections import deque
from pathlib import Path
from multiprocessing import Process, Value, Event
import gpiod
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
from datetime import datetime, date
import numpy as np
import os, sys, time
import serial
import random

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src'))

import config
import state_machine
import motor_ctrl
import servo_ctrl
import camera_ctrl
import airspeed_sensor_ctrl
import plot_motor_data
import plot_li550_data
import calibration_file_handling


class PwmChannel:
    def __init__(self, name, pin):
        self.name = name
        self.pin = pin
        self.line = None
        self.last_rising = None
        self.pulse = 0.0
        self.timeout = False

    def __repr__(self):
        return f"<PwmChannel {self.name}: pulse={self.pulse:.1f}µs timeout={self.timeout}>"


def _request_line(chip: gpiod.Chip, pin: int) -> gpiod.Line:
    line = chip.get_line(pin)
    line.request(consumer='pwm_reader', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
    return line


def _update_channel_from_event(ch: PwmChannel, wait_ok: bool):
    """Mutates channel in place based on whether an event occurred."""
    if wait_ok:
        ev = ch.line.event_read()
        ts = ev.sec + ev.nsec / 1e9
        if ev.type == gpiod.LineEvent.RISING_EDGE:
            ch.last_rising = ts
        elif ev.type == gpiod.LineEvent.FALLING_EDGE and ch.last_rising is not None:
            ch.pulse = (ts - ch.last_rising) * 1_000_000  # µs
            ch.last_rising = None
        ch.timeout = False
    else:
        ch.pulse = 0.0
        ch.timeout = True


def rc_receiver_reading(shared_remote_command, shared_target_speed, shared_wheel_position, shared_knobl_value, shared_knobr_value, restart_event):
    try:
        with gpiod.Chip('gpiochip0') as chip:
            # Set up all channels in one place
            channels: dict[str, PwmChannel] = {
                "throttle": PwmChannel("throttle", config.TROTTLE_PIN),
                "button":   PwmChannel("button",   config.BUTTON_PIN),
                "steering": PwmChannel("steering", config.STEERING_PIN),
                "switch":   PwmChannel("switch",   config.SWITCH_PIN),
                "knobl":    PwmChannel("knobl",    config.KNOBL_PIN),
                "knobr":    PwmChannel("knobr",    config.KNOBR_PIN),
            }

            # Request all lines
            for ch in channels.values():
                ch.line = _request_line(chip, ch.pin)

            # Locals for higher-level logic/state
            remote_command = 0
            last_remote_command = 0
            wheel_position = 0
            target_speed = 0.0
            knobl_value = 0.0
            knobr_value = 0.0
            knobs_initialized = False
            knobl_history = deque(maxlen=config.KNOB_MEDIAN_WINDOW)
            knobr_history = deque(maxlen=config.KNOB_MEDIAN_WINDOW)
            last_t0 = None

            # Button initialization to detect the toggled position relative to startup
            initial_button_pulse = 0.0
            button_initialized = False
            button_in_default_position = True

            while not restart_event.is_set():
                t0 = time.time()
                dt_rc = config.DT_RC if last_t0 is None else t0 - last_t0
                last_t0 = t0

                # Wait for events (1s timeout each) and update all channels in a loop
                for key in ("throttle", "button", "steering", "switch", "knobl", "knobr"):
                    ch = channels[key]
                    had_event = ch.line.event_wait(sec=1)
                    _update_channel_from_event(ch, had_event)

                # Optional debug prints
                # print("throttle_pulse:", channels["throttle"].pulse)
                # print("steering_pulse:", channels["steering"].pulse)
                # print("button_pulse:",   channels["button"].pulse)
                # print("switch_pulse:", channels["switch"].pulse)
                # print("knobl_pulse:",  channels["knobl"].pulse)
                # print("knobr_pulse:",  channels["knobr"].pulse)
                # print("---")

                # Shorthand variables
                th = channels["throttle"]
                bt = channels["button"]
                st = channels["steering"]
                sw = channels["switch"]
                kl = channels["knobl"]
                kr = channels["knobr"]

                # Disconnection / timeout handling
                if (th.timeout or th.pulse == 0.0 or
                    bt.timeout or bt.pulse == 0.0 or
                    st.timeout or st.pulse == 0.0 or
                    sw.timeout or sw.pulse == 0.0 or
                    kl.timeout or kl.pulse == 0.0 or
                    kr.timeout or kr.pulse == 0.0):
                    if last_remote_command == 3:
                        # stay in tracking but stop
                        remote_command = 3
                        target_speed = 0.0
                        wheel_position = 0
                    else:
                        # fall back to stop
                        remote_command = 0
                        target_speed = 0.0
                        wheel_position = 0
                else:
                    # Initialize button reference the first time we get a valid pulse
                    if not button_initialized:
                        initial_button_pulse = bt.pulse
                        button_initialized = True

                    # Is the button still in its initial position?
                    if button_initialized and abs(bt.pulse - initial_button_pulse) < config.BUTTON_TOGGLE_THRESHOLD:
                        button_in_default_position = True
                    else:
                        button_in_default_position = False

                    # Remote command / speed logic
                    if remote_command == 3 or remote_command == 4:
                        # Tracking mode
                        remote_command = 3
                        target_speed = 0.0

                        # Take off mode: the switch on its low end, tracking having been entered
                        # by toggling the button (either order works)
                        if sw.pulse < config.PWM_MIN_PULSE_WIDTH:
                            remote_command = 4

                        # Safety: stop if button is toggled or not in default position
                        if button_in_default_position or abs(th.pulse - config.PWM_DEFAULT_PULSE_WIDTH) > config.GO_STOP_THRESHOLD:
                            remote_command = 0
                            target_speed = 0.0
                            # require a toggle to resume tracking
                            initial_button_pulse = bt.pulse
                    else:
                        # Manual mode
                        if not button_in_default_position:
                            remote_command = 3
                            target_speed = 0.0
                        else:
                            delta = th.pulse - config.PWM_DEFAULT_PULSE_WIDTH
                            if abs(delta) < config.GO_STOP_THRESHOLD:
                                remote_command = 0   # GO_STOP
                                target_speed = 0.0
                            elif delta < -config.GO_STOP_THRESHOLD:
                                remote_command = 1   # GO_BACKWARD
                                target_speed = th.pulse
                            else:  # delta > +threshold
                                remote_command = 2   # GO_FORWARD
                                target_speed = th.pulse

                    # Calibration setpoints from steering
                    if abs(st.pulse - config.PWM_MIN_PULSE_WIDTH) < config.CALIB_SETPOINTS_THRESHOLD:
                        wheel_position = 1  # zipline start
                    elif abs(st.pulse - config.PWM_MAX_PULSE_WIDTH) < config.CALIB_SETPOINTS_THRESHOLD:
                        wheel_position = 2  # zipline length
                    else:
                        wheel_position = 0  # no setpoint

                    # Knob values (0.0 to 1.0)
                    knobl_raw = (kl.pulse - config.PWM_MIN_PULSE_WIDTH) / (config.PWM_MAX_PULSE_WIDTH - config.PWM_MIN_PULSE_WIDTH)
                    knobr_raw = (kr.pulse - config.PWM_MIN_PULSE_WIDTH) / (config.PWM_MAX_PULSE_WIDTH - config.PWM_MIN_PULSE_WIDTH)

                    knobl_raw = np.clip(knobl_raw, 0.0, 1.0)
                    knobr_raw = np.clip(knobr_raw, 0.0, 1.0)

                    # The knobs are static analog inputs, but the measured pulse widths carry a few µs
                    # of jitter with the odd 40 µs outlier (~0.6 m/s on the take-off release threshold,
                    # which is knobl * MAX_SPEED). The median rejects the outliers, the low pass smooths
                    # what is left; both are far quicker than a hand can turn a knob.
                    knobl_history.append(knobl_raw)
                    knobr_history.append(knobr_raw)
                    knobl_median = float(np.median(knobl_history))
                    knobr_median = float(np.median(knobr_history))

                    if not knobs_initialized:
                        # Start on the first reading, otherwise the filter ramps up from 0.0
                        knobl_value = knobl_median
                        knobr_value = knobr_median
                        knobs_initialized = True
                    else:
                        knobl_value = motor_ctrl.low_pass(knobl_median, knobl_value, config.CUT_OFF_FREQUENCY_KNOB, dt_rc)
                        knobr_value = motor_ctrl.low_pass(knobr_median, knobr_value, config.CUT_OFF_FREQUENCY_KNOB, dt_rc)

                last_remote_command = remote_command

                # Share results atomically
                with (  shared_remote_command.get_lock(),
                        shared_target_speed.get_lock(),
                        shared_wheel_position.get_lock(),
                        shared_knobl_value.get_lock(),
                        shared_knobr_value.get_lock()):

                    shared_remote_command.value = remote_command
                    shared_target_speed.value = target_speed
                    shared_wheel_position.value = wheel_position
                    shared_knobl_value.value = knobl_value
                    shared_knobr_value.value = knobr_value

                # Loop timing
                dt = time.time() - t0
                if dt < config.DT_RC:
                    time.sleep(config.DT_RC - dt)
                else:
                    print(f"RC process execution time exceeded: {dt:.4f} / {config.DT_RC:.4f} s.")

    except KeyboardInterrupt:
        print("\nRC process stopped.")
    finally:
        # Release lines if they were requested
        try:
            for ch in list(locals().get("channels", {}).values()):
                if ch.line is not None:
                    ch.line.release()
        except Exception:
            pass


def wind_sensor_reading(save_path, time_start_ref, restart_event): 
    # Data recording setings
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = Path(save_path) / f"wind_data_{timestamp}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = open(csv_path, 'w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=config.CSV_WIND_COLUMNS)
    writer.writeheader()

    try:
        ser = serial.Serial(config.SERIAL_PORT_LI550, config.BAUD_RATE_LI550, timeout=0.05)  # short: log_li550_data's read(4096) shouldn't block the loop for long
        while not restart_event.is_set():
            time_start_while = time.time()

            # Read data from LI550
            li550_data = airspeed_sensor_ctrl.log_li550_data(ser)

            # Save data to CSV (close() below flushes the buffer on exit, so nothing is lost on a clean stop)
            row = {'Unix Timestamp [s]': np.round(time.time(), 3), **li550_data}
            writer.writerow(row)

            # Sleep to respect the desired loop time
            time_end_while = time.time()
            # print("Wind process:", time_end_while - time_start_while, "s")
            if time_end_while - time_start_while < config.DT_WIND:
                time.sleep(config.DT_WIND - (time_end_while - time_start_while))
            else:
                print(f"Wind process execution time exceeded: {(time_end_while - time_start_while):.4f} / {config.DT_WIND:.4f} s.")

    except KeyboardInterrupt:
        print("\nWind sensor process stopped.")

    finally:
        csv_file.close()
        print(f"\nRun complete. Wind data saved to {csv_path}")
        try:
            plot_li550_data.plot_data(csv_path)
            # plot_li550_data.create_video_from_data(csv_path)
            print("Plotting wind data complete")
        except Exception as e:
            print(f"Plotting failed: {e}")


def make_take_off_trajectory():
    # Create a take-off trajectory that allows user to sync up
    dt = config.DT_MAIN
    N_init = int(5/dt)
    N = int(1/dt)  # number of steps for each phase
    p = 0.75 # [m]
    v_f = config.MAX_SPEED_CALIB
    v_s = 0.5
    return [0]*N_init + [p]*N + [0]*N + [p]*N + [0]*N + [p]*N + [0]*N, [0]*N_init + [v_f]*N + [v_s]*N + [v_f]*N + [v_s]*N + [v_f]*N + [v_s]*N


def main(save_path, time_start_ref, shared_remote_command, shared_target_speed, shared_wheel_position, shared_knobl_value, shared_knobr_value, restart_event):
    # Imported here and not at module level: it pulls in lgpio through blinka, and lgpio only
    # generates servo pulses in the process that initialized it (the parent would break the servo)
    import leds_ctrl

    # Initialize variable
    time_start_while = 0
    time_end_while = 0
    leds_off_before = False
    state = config.STATE["STOP"]
    last_state = state
    x_stop = 0
    start_decelerating = True
    decelerating = True

    tracking_error = None
    last_tracking_error = None
    last_target_position = None
    last_time_frame_captured = None
    frame_counter = 1
    estimated_UAV_vel = 0.0
    last_estimated_UAV_vel = 0.0
    estimated_UAV_pos = 0.0
    last_estimated_UAV_pos = 0.0
    cnt_moving_blindly = 0

    knobr_prev = 0.0

    in_calibration_mode = True
    zipline_end_set = False
    zipline_length_loaded = False
    zipline_length = 0
    zipline_start = 0
    zipline_end = 0
    last_wheel_position = 0

    restart_counter = 0

    # Camera settings
    cal = camera_ctrl.load_calibration()
    if isinstance(cal, (list, tuple)) and len(cal) >= 2:
        mtx, dist = cal[0], cal[1]
    else:
        raise RuntimeError("camera_ctrl.load_calibration() must return at least (mtx, dist).")

    pipeline = camera_ctrl.ArucoPipeline(
        mtx, dist,
        alpha=getattr(config, "UNDISTORT_ALPHA", 0.0),
        new_size=getattr(config, "UNDISTORT_SIZE", None)
    )

    # Initialize peripherals
    leds = leds_ctrl.leds_init()
    odrv = motor_ctrl.motor_init()
    camera = camera_ctrl.camera_init()
    servo = servo_ctrl.servo_init()
    if servo is None:
        print("WARNING: servo not available, the launcher will not release")

    # Try to load calibration data from today's file
    calibration_data = calibration_file_handling.load_calibration_data()
    if calibration_data is not None:
        zipline_length = calibration_data
        zipline_end_set = True # Skip setting the end of the zipline in the calibration mode
        zipline_length_loaded = True
        print(f"Using saved calibration: length={zipline_length:.2f}m")

    # Motor settings for calibration
    odrv.axis0.pos_estimate = motor_ctrl.linear_to_angular(-config.INITIAL_MOTOR_POS_CALIB)
    odrv.axis0.trap_traj.config.vel_limit = motor_ctrl.linear_to_angular(config.MAX_SPEED_CALIB)  
    x_ref = config.INITIAL_MOTOR_POS_CALIB
    last_x_ref = config.INITIAL_MOTOR_POS_CALIB
    vel_ref = 0.0
    last_vel_ref = 0.0

    # Data recording setings 
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = Path(save_path) / f"data_{timestamp}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = open(csv_path, 'w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=config.CSV_COLUMNS)
    writer.writeheader()
    save_camera_path = Path(save_path) / "frames"
    save_camera_path.mkdir(parents=True, exist_ok=True)

    # Variables for take off logic
    take_off_done = True
    take_off_direction = None
    take_off_start_position = None
    take_off_i = None
    take_off_acc = config.TAKE_OFF_ACCELERATION
    take_off_acc_history = deque(maxlen=config.TAKE_OFF_ACC_MEDIAN_WINDOW)
    take_off_last_vel = None
    take_off_last_time = None
    take_off_poss, take_off_vels = make_take_off_trajectory()
    servo_release_time = None
    servo_position = config.SERVO_START_POSITION
    servo_stop_time = time.time() + servo_ctrl.TRAVEL_TIME

    # Start video recording
    if config.SAVE_IMAGES == 3:
        encoder = H264Encoder(bitrate=None, repeat=True, framerate=1)
        mp4_path = save_path / "run.mp4"
        camera.start_recording(encoder, FfmpegOutput(str(mp4_path)))

    try:
        try:
            while True:
                time_start_while = time.time()

                # Move the servo back to its start position (timed, as blocking would freeze the loop)
                if (servo_release_time is not None) and (time_start_while - servo_release_time >= config.SERVO_HOLD_TIME):
                    servo_position = config.SERVO_START_POSITION
                    servo_ctrl.set_position(servo, servo_position)
                    servo_release_time = None
                    servo_stop_time = time_start_while + servo_ctrl.TRAVEL_TIME
                    print(f"Servo back to start position ({servo_position})")

                # Stop the pulses once the servo has travelled, a servo that is not driven cannot jitter
                if (servo_stop_time is not None) and (time_start_while >= servo_stop_time):
                    servo_ctrl.stop(servo)
                    servo_stop_time = None

                # Check if there is an error on the ODrive
                if (odrv.axis0.active_errors != 0 or (odrv.axis0.disarm_reason != 0)):
                    print(f"ODrive error: {odrv.axis0.active_errors}  Disarm reason: {odrv.axis0.disarm_reason}")
                    print("- Error 64: Missing input")
                    print("- Error 512: DC bus undervoltage")
                    print("- Error 256: DC bus overvoltage")
                    print("- Error 1024: DC overcurrent")

                    leds_off_before = leds_ctrl.leds_error_warning(leds, leds_off_before)
                    odrv.axis0.controller.input_pos = odrv.axis0.pos_estimate # stay in the same position
                else:
                    # Get remote control commands
                    with shared_remote_command.get_lock(), shared_target_speed.get_lock(), shared_wheel_position.get_lock():
                        remote_command = shared_remote_command.value      # 0: no command, 1: go backward, 2: go forward, 3: go tracking, 4: take off
                        target_speed = shared_target_speed.value          # in µs
                        wheel_position = shared_wheel_position.value      # 0: no setpoint, 1: zipline start, 2: zipline length
                        knobl_value = shared_knobl_value.value            # 0.0 to 1.0
                        knobr_value = shared_knobr_value.value            # 0.0 to 1.0

                        # Map target_speed (µs) to target_speed_m_s (m/s)
                        speed_max = config.MAX_SPEED_CALIB if in_calibration_mode else config.MAX_SPEED
                        if remote_command == 1:
                            target_speed_m_s = speed_max - np.interp(
                                target_speed,
                                [config.PWM_MIN_PULSE_WIDTH, config.PWM_DEFAULT_PULSE_WIDTH - config.GO_STOP_THRESHOLD],
                                [0.0, speed_max])
                        elif remote_command == 2:
                            target_speed_m_s = np.interp(
                                target_speed,
                                [config.PWM_DEFAULT_PULSE_WIDTH + config.GO_STOP_THRESHOLD, config.PWM_MAX_PULSE_WIDTH],
                                [0.0, speed_max])
                        else:
                            target_speed_m_s = 0.0

                    # Update state
                    if (remote_command == config.REMOTE_COMMAND["GO_STOP"]) and (not in_calibration_mode):
                        if wheel_position > 0: # full clockwise or counterclockwise to trigger the restart
                            restart_counter += config.DT_MAIN
                            if restart_counter >= config.RESTART_HOLD_TIME:
                                print("Restart triggered")
                                leds_ctrl.leds_restart(leds)
                                restart_event.set()   # <— tell parent to re-exec
                                break                 # <— drop to finally (cleanup & save)
                        else:
                            restart_counter = 0

                    state = state_machine.update(last_state, remote_command, in_calibration_mode)
                    last_state = state
                    
                    # Get motor data                    
                    angular_position, angular_velocity, torque, linear_position, linear_velocity, voltage, current, decel_dist = motor_ctrl.get_data(odrv) # in turns, turns/s, Nm, m, m/s
                    time_position_measured = time.time()

                    # Calibrating the zipline length
                    if in_calibration_mode:
                        print(f"CALIBRATION  Position: {linear_position:.2f} m  |  Velocity: {linear_velocity:.2f} m/s {x_ref:.2f}")

                        # Display the calibration mode with LEDs
                        leds_off_before = leds_ctrl.leds_set_color_calibration(leds, leds_off_before)
                        
                        # Setting the end of the zipline (on the wheel moving into the end setpoint,
                        # so that the end can be set again to trigger a new measurement)
                        if (wheel_position == 2) and (last_wheel_position != 2):
                            zipline_end = linear_position
                            zipline_end_set = True
                            zipline_length_loaded = False # A newly set end overrides the loaded length
                            print(f"Zipline end set: {zipline_end:.2f} m")
                            leds_ctrl.leds_show_setpoint_calibration(leds)
                            time.sleep(1)  # Hold the purple light to confirm the end was registered
                        
                        # Setting the start of the zipline
                        if (wheel_position == 1) and (last_wheel_position != 1) and zipline_end_set:
                            if not zipline_length_loaded:
                                # If the end was set in this run, compute the zipline length from the start and end
                                # positions. Otherwise keep the length loaded from the calibration file.
                                zipline_start = linear_position
                                zipline_length = zipline_end - zipline_start

                            # Validate the calibration
                            if zipline_length <= 0:
                                print(f"ERROR: Invalid zipline length: {zipline_length:.2f} m. Start position must be less than end position.")
                                # Flash error pattern on LEDs
                                leds_ctrl.leds_error_warning(leds, True)
                                time.sleep(3)
                                # Reset calibration
                                zipline_end_set = False
                            else:
                                # Motor settings for normal operation
                                odrv.axis0.pos_estimate = motor_ctrl.linear_to_angular(0)
                                odrv.axis0.trap_traj.config.vel_limit = motor_ctrl.linear_to_angular(config.MAX_SPEED)
                                linear_position = 0
                                x_ref = 0
                                last_x_ref = 0                        
                                in_calibration_mode = False # Calibration done   

                                # Save calibration data to file
                                calibration_file_handling.save_calibration_data(zipline_length)
                                print(f"Calibration done: zipline length: {zipline_length:.2f} m")
                                leds_ctrl.leds_show_setpoint_calibration(leds)
                                time.sleep(1)  # Hold the purple light to confirm the start was registered

                        last_wheel_position = wheel_position

                        # Desired velocity based on the remote command
                        vel_ref = target_speed_m_s

                        if state == config.STATE["STOP"]:
                            x_ref = linear_position                          
                        elif state == config.STATE["FORWARD"]:
                            x_ref = config.ZIPLINE_LENGTH_CALIB
                        elif state == config.STATE["BACKWARD"]:
                            x_ref = config.ZIPLINE_START_CALIB
                        else:
                            x_ref = linear_position

                        # Set the target position and velocity of the motor
                        motor_ctrl.set_pos_vel(odrv, x_ref, vel_ref)
                        
                    # Normal operation
                    else:
                        # MANUAL MODE
                        if state == config.STATE["STOP"] or state == config.STATE["FORWARD"] or state == config.STATE["BACKWARD"]:
                            take_off_i = None

                            ArUco_pose = {
                                'tvec_x [m]': None,
                                'tvec_y [m]': None,
                                'tvec_z [m]': None,
                                'rvec_x [rad]': None,
                                'rvec_y [rad]': None,
                                'rvec_z [rad]': None,
                                'qx': None,
                                'qy': None,
                                'qz': None,
                                'qw': None,
                            }

                            tracking_error = None
                            last_tracking_error = None
                            v_tracking_error = None

                            # Desired velocity based on the remote command
                            vel_ref = target_speed_m_s

                            if state == config.STATE["STOP"]:
                                # Define positon target to match desired deceleration with respect to current velocity
                                if start_decelerating:
                                    if linear_velocity > 0:
                                        x_stop = linear_position + decel_dist
                                    else:
                                        x_stop = linear_position - decel_dist
                                    start_decelerating = False

                                # Stay on spot when position target for deceleration reached   
                                decelerating = True
                                if abs(linear_velocity) < config.STOP_SPEED_THRESHOLD: decelerating = False
                                if decelerating:
                                    x_ref = x_stop
                                else:
                                    x_ref = linear_position
                                                            
                            elif state == config.STATE["FORWARD"]:
                                x_ref = zipline_length
                                start_decelerating = True

                            elif state == config.STATE["BACKWARD"]:
                                x_ref = 0
                                start_decelerating = True

                            else:
                                x_ref = linear_position
                                start_decelerating = True

                            # To switch from FORWARD to TRACKING smoothly
                            last_estimated_UAV_vel = linear_velocity
                        
                        # TRACKING MODE
                        elif (state == config.STATE["TRACKING"]) or (state == config.STATE["TAKE_OFF"]):                     
                            # Take frame 
                            frame_bgr = camera_ctrl.capture_bgr_frame(camera)
                            time_frame_captured = time.time()

                            ArUco_pose, duration_process, _ = pipeline.process_bgr_frame(
                                frame_bgr,
                                time_frame_captured,
                                save_path=str(save_camera_path),
                                frame_counter=frame_counter,
                                time_start_ref=time_start_ref
                            )
                            frame_counter += 1

                            if ArUco_pose['tvec_x [m]'] is not None:
                                tracking_error = - ArUco_pose['tvec_x [m]']
                            else:
                                tracking_error = None

                            if tracking_error is not None:
                                # ArUco marker detected
                                tracking_error = - ArUco_pose['tvec_x [m]']
                                estimated_UAV_pos = linear_position + tracking_error
                                cnt_moving_blindly = 0
        
                                time_diff = None if last_time_frame_captured is None else time_frame_captured - last_time_frame_captured

                                # A longer gap means we are re-acquiring the tag: differentiating across it
                                # would average out everything that happened in between
                                fresh_detection = (time_diff is not None) and (0 < time_diff <= config.MAX_TRACKING_TIME_GAP)

                                if fresh_detection:
                                    estimated_UAV_vel = (estimated_UAV_pos - last_estimated_UAV_pos) / time_diff
                                    estimated_UAV_vel = motor_ctrl.low_pass(estimated_UAV_vel, last_estimated_UAV_vel, config.CUT_OFF_FREQUENCY_TRACKING, time_diff)
                                else:
                                    estimated_UAV_vel = last_estimated_UAV_vel

                                last_estimated_UAV_vel = estimated_UAV_vel

                                # v_tracking_error = 0.0 if last_tracking_error is None else (tracking_error - last_tracking_error) / (time_frame_captured - last_time_frame_captured)
                                if fresh_detection and (last_tracking_error is not None) and (v_tracking_error is not None):
                                    v_tracking_error = motor_ctrl.low_pass((tracking_error - last_tracking_error) / time_diff, v_tracking_error, config.CUT_OFF_FREQUENCY_TRACKING, time_diff)
                                else:
                                    v_tracking_error = 0.0

                                last_time_frame_captured = time_frame_captured
                                last_estimated_UAV_pos = estimated_UAV_pos
                                last_tracking_error = tracking_error

                                start_length = zipline_length*config.TAKE_OFF_POSITION_THRESHOLD
                                if linear_position <= start_length:
                                    offset = config.STARTING_OFFSET*linear_position/start_length
                                elif linear_position >= zipline_length - start_length:
                                    offset = -config.STARTING_OFFSET*(linear_position - (zipline_length - start_length))/start_length
                                else:
                                    offset = 0.0
                                p_part = config.P_GAIN_VISION*(tracking_error + offset)
                                d_part = np.clip(config.D_GAIN_VISION*v_tracking_error, -np.abs(p_part), np.abs(p_part)) # avoid overshooting due to derivative part when UAV is very close
                                vel_ref = estimated_UAV_vel + p_part - d_part
                                
                                if (vel_ref == 0):
                                    x_ref = linear_position
                                if (vel_ref > 0):
                                    x_ref = zipline_length
                                else:
                                    x_ref = 0
                            else:
                                # ArUco marker not detected: keep the last tracking_error for a while
                                if cnt_moving_blindly < config.MAX_CNT_MOVING_BLINDLY:

                                    vel_ref = last_estimated_UAV_vel

                                    if (vel_ref == 0):
                                        x_ref = linear_position
                                    elif (vel_ref > 0):
                                        x_ref = zipline_length
                                    else:
                                        x_ref = 0
                                    
                                    cnt_moving_blindly += 1
                                else:
                                    # No ArUco marker detected and no previous offset: stop the motor
                                    x_ref = linear_position
                                    vel_ref = 0
                                    estimated_UAV_pos = x_ref

                            if state == config.STATE["TAKE_OFF"]:
                                # first time
                                if take_off_i is None:
                                    take_off_i = 0
                                    if linear_position < zipline_length*config.TAKE_OFF_POSITION_THRESHOLD:
                                        take_off_direction = "FORWARD"
                                    elif linear_position > zipline_length*(1-config.TAKE_OFF_POSITION_THRESHOLD):
                                        take_off_direction = "BACKWARD"
                                    else:
                                        take_off_direction = None
                                    take_off_start_position = linear_position
                                    take_off_acc = config.TAKE_OFF_ACCELERATION
                                    take_off_acc_history.clear()
                                    take_off_last_vel = None
                                    take_off_last_time = None

                                # Acceleration of this run, estimated live because the trapezoidal profile
                                # does not hold a constant value. Clamped at 0 to ignore the reversal at the
                                # start of the run (the cart is still rolling backwards out of the last
                                # sync-up leg) and low passed to take the edge off differentiating at DT_MAIN.
                                # The median matters more than it looks: the estimate sets how far ahead the
                                # release fires, so one short dt or one bad velocity sample would otherwise
                                # let a spuriously high value release the UAV a metre per second early.
                                if (take_off_last_time is not None) and (time_position_measured > take_off_last_time):
                                    dt_take_off = time_position_measured - take_off_last_time
                                    take_off_acc_history.append(np.clip((abs(linear_velocity) - take_off_last_vel) / dt_take_off, 0.0, config.TAKE_OFF_MAX_ACCELERATION))
                                    take_off_acc = motor_ctrl.low_pass(float(np.median(take_off_acc_history)), take_off_acc, config.CUT_OFF_FREQUENCY_TAKE_OFF, dt_take_off)
                                take_off_last_vel = abs(linear_velocity)
                                take_off_last_time = time_position_measured

                                # When to command the servo so the latch opens at the requested speed.
                                # linear_velocity was measured at the top of the loop and the camera capture
                                # and ArUco pipeline ran in between, so extrapolate it to now, then work out
                                # how long until the cart reaches the target and subtract the latch's own
                                # opening time.
                                take_off_speed_target = knobl_value * config.MAX_SPEED
                                time_now = time.time()
                                take_off_velocity = abs(linear_velocity) + take_off_acc*(time_now - time_position_measured)
                                if take_off_acc > 0.1:
                                    time_to_release = (take_off_speed_target - take_off_velocity)/take_off_acc - config.SERVO_RELEASE_DELAY
                                else:
                                    # Not accelerating (yet): fall back to a plain comparison so a stalled
                                    # run cannot divide by ~0 and release on the spot.
                                    time_to_release = 0.0 if take_off_velocity >= take_off_speed_target else np.inf

                                # The cart gains ~0.7 m/s per loop, so the release moment almost never lands
                                # on an iteration boundary. Fire on the last iteration before it passes:
                                # with TAKE_OFF_PRECISE_RELEASE the remainder is slept off below, without it
                                # the half period centres the error instead of always overshooting.
                                take_off_release_due = time_to_release <= (config.DT_MAIN if config.TAKE_OFF_PRECISE_RELEASE else config.DT_MAIN/2)

                                # trajectory that allows user to sync up
                                if take_off_i < len(take_off_poss):
                                    if take_off_direction == "FORWARD":
                                        x_ref = take_off_start_position + take_off_poss[take_off_i]
                                        vel_ref = take_off_vels[take_off_i]
                                    elif take_off_direction == "BACKWARD":
                                        x_ref = take_off_start_position - take_off_poss[take_off_i]
                                        vel_ref = take_off_vels[take_off_i]
                                    else:
                                        take_off_i = len(take_off_poss) # skip to the end
                                        x_ref = linear_position
                                        vel_ref = 0
                                    take_off_i += 1

                                    take_off_done = False
                                
                                # timed take off
                                elif (not take_off_release_due) and (not take_off_done):
                                    print(f"TAKE_OFF accelerating: {take_off_velocity:.2f} / {take_off_speed_target:.2f} m/s  (a={take_off_acc:.1f} m/s², release in {time_to_release:.3f} s)")
                                    cnt_moving_blindly = 0
                                    last_estimated_UAV_vel = linear_velocity
                                    if take_off_direction == "FORWARD":
                                        x_ref = zipline_length
                                        vel_ref = config.MAX_SPEED
                                    elif take_off_direction == "BACKWARD":
                                        x_ref = 0
                                        vel_ref = config.MAX_SPEED
                                    else:
                                        take_off_done = True # skip to the end

                                else:
                                    # Release the UAV at the end of the take-off maneuver
                                    if not take_off_done:
                                        # Wait out the fraction of a loop period left before the cart is at
                                        # the requested speed, so the latch opens on it rather than at the
                                        # next iteration boundary. Bounded by one loop period and run once
                                        # per take-off; the ODrive keeps executing its trajectory throughout.
                                        if config.TAKE_OFF_PRECISE_RELEASE:
                                            wait_before_release = np.clip(time_now + time_to_release - time.time(), 0.0, config.DT_MAIN)
                                            if wait_before_release > 0:
                                                time.sleep(wait_before_release)

                                        servo_position = config.SERVO_END_POSITION
                                        servo_ctrl.set_position(servo, servo_position)
                                        servo_release_time = time.time()
                                        servo_stop_time = None
                                        expected_release_velocity = take_off_velocity + take_off_acc*(servo_release_time + config.SERVO_RELEASE_DELAY - time_now)
                                        print(f"Servo release ({servo_position}) commanded at {take_off_velocity:.2f} m/s, latch expected to open at {expected_release_velocity:.2f} / {take_off_speed_target:.2f} m/s")

                                    take_off_done = True
                                    state = config.STATE["TRACKING"]
                                    last_state = config.STATE["TAKE_OFF"]

                            start_decelerating = True

                        else:
                            print("ERROR: unknown state")

                        # Update exposure time
                        if abs(knobr_prev - knobr_value) > 0.1:
                            knobr_prev = knobr_value
                            exposure_time = config.EXPOSURE_TIME_MIN + knobr_value * (config.EXPOSURE_TIME_MAX - config.EXPOSURE_TIME_MIN)
                            camera_ctrl.set_exposure_manually(camera, exposure_time)
                        
                        # Save data to CSV
                        motor_data = motor_ctrl.log_motor_data(time.time(), angular_position, angular_velocity, torque, linear_position, linear_velocity, tracking_error, voltage, current, x_ref, estimated_UAV_pos, estimated_UAV_vel, vel_ref)
                        row = {**motor_data, **ArUco_pose}
                        writer.writerow(row)   

                        # Print current state
                        offset_str = "N/A" if tracking_error is None else f"{tracking_error:.2f} m"
                        log_message = (f"Cmd: {config.COMMAND_LOOKUP.get(remote_command, 'UNKNOWN')}  |  "
                                       f"State: {config.STATE_LOOKUP.get(state, 'UNKNOWN')}  |   "
                                       f"Pos: {linear_position:.2f} m  |  "
                                       f"Vel: {linear_velocity:.2f} m/s  |  "
                                       f"ArUco offset: {offset_str} m  |  "
                                       f"x_ref: {x_ref} m")
                        # print(log_message)

                        # Set the target position and velocity of the motor
                        x_ref = np.clip(x_ref, 0, zipline_length)
                        vel_ref = np.clip(abs(vel_ref), 0, config.MAX_SPEED)
                        motor_ctrl.set_pos_vel(odrv, x_ref, vel_ref)

                        last_x_ref = x_ref
                        last_vel_ref = vel_ref

                        # Display current state with LEDs
                        leds_off_before = leds_ctrl.leds_set_color(leds, state, tracking_error, leds_off_before, in_calibration_mode)

                # Sleep to respect the desired loop time
                time_end_while = time.time()
                # print("Main process:", time_end_while - time_start_while, "s") # 0.011 s without camera on, 0.075 s with camera but no saving
                if time_end_while - time_start_while < config.DT_MAIN:
                    time.sleep(config.DT_MAIN - (time_end_while - time_start_while))
                else:
                    print(f"Main process execution time exceeded: {(time_end_while - time_start_while):.4f} / {config.DT_MAIN:.4f} s.")

        except KeyboardInterrupt:
            print("\nMain process stopped.")
            odrv.axis0.requested_state = 1  # Set ODrive to idle state
            leds.fill((0, 0, 0)) # Turn off LEDs
            leds.show()

    finally:
        servo_ctrl.set_position(servo, config.SERVO_START_POSITION)
        time.sleep(0.5)  # let the servo travel back before it stops being driven
        servo_ctrl.servo_deinit(servo)

        if config.SAVE_IMAGES == 3:
            camera.stop_recording()
        csv_file.close()
        print(f"\nRun complete. Data saved to {csv_path}")
        try:
            plot_motor_data.plot_data(csv_path)
            print("Plotting motor data complete")
            image_path = save_path / "frames"
            camera_ctrl.images_to_mp4(image_path, output_path=save_path, fps=1/config.DT_MAIN)
        except Exception as e:
            print(f"Plotting failed: {e}")


if __name__ == "__main__":
    # Create a folder to save the data
    timestamp_folder = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_path = Path("data") / f"run_{timestamp_folder}"
    os.makedirs(save_path, exist_ok=True)
    time_start_ref = time.time()

    restart_event = Event()

    # Shared variables from RC process to main process
    shared_remote_command = Value('i', 0)
    shared_target_speed = Value('d', 0.0)
    shared_wheel_position = Value('d', 0.0)
    shared_knobl_value = Value('d', 0.0)
    shared_knobr_value = Value('d', 0.0)

    p1 = Process(
        target=rc_receiver_reading,
        args=(shared_remote_command,
              shared_target_speed,
              shared_wheel_position,
              shared_knobl_value,
              shared_knobr_value,
              restart_event),
    )
    p3 = Process(
        target=main,
        args=(save_path,
              time_start_ref,
              shared_remote_command,
              shared_target_speed,
              shared_wheel_position,
              shared_knobl_value,
              shared_knobr_value,
              restart_event),
    )

    procs = [p1, p3]

    if config.ENABLE_WIND_SENSING:
        p2 = Process(
            target=wind_sensor_reading,
             args=(save_path,
                   time_start_ref,
                   restart_event),
        )
        procs.append(p2)

    # start everything
    for p in procs:
        p.start()

    def shutdown_children():
        """
        Wait for the children to run their own shutdown (closing the CSVs, plotting the data),
        which is why they are joined rather than terminated straight away. Terminating them
        0.5 s into a restart is what used to skip the plots.
        """
        deadline = time.time() + config.SHUTDOWN_TIMEOUT
        for p in procs:
            p.join(timeout=max(0.0, deadline - time.time()))
        for p in procs:
            if p.is_alive():
                print(f"{p.name} did not exit in time, terminating.")
                p.terminate()
        for p in procs:
            p.join()

    # Parent supervisor loop: if restart requested, re-exec the script
    try:
        while any(p.is_alive() for p in procs):
            if restart_event.is_set():
                shutdown_children()

                # Re-exec the current Python process (fresh run, new save_path, etc.)
                os.execv(sys.executable, [sys.executable] + sys.argv)

            time.sleep(0.2)

        # normal exit if all children ended without restart
        sys.exit(0)

    except KeyboardInterrupt:
        shutdown_children()
        sys.exit(0)