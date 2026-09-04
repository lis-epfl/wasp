import sys
import time

import config
import servo_ctrl


def test_endpoints(servo):
    """
    Move the servo back and forth between its start and end position
    :param servo: GPIO chip handle
    """
    print(f"Start position: {config.SERVO_START_POSITION}")
    servo_ctrl.set_position(servo, config.SERVO_START_POSITION)
    time.sleep(1.0)

    print(f"End position: {config.SERVO_END_POSITION}")
    servo_ctrl.set_position(servo, config.SERVO_END_POSITION)
    time.sleep(1.0)

    print(f"Start position: {config.SERVO_START_POSITION}")
    servo_ctrl.set_position(servo, config.SERVO_START_POSITION)
    time.sleep(1.0)


def test_release(servo):
    """
    Replay the take-off release sequence used in main.py
    :param servo: GPIO chip handle
    """
    servo_ctrl.set_position(servo, config.SERVO_START_POSITION)
    time.sleep(1.0)

    print("Release")
    servo_ctrl.set_position(servo, config.SERVO_END_POSITION)
    time.sleep(config.SERVO_HOLD_TIME)

    print("Back to start position")
    servo_ctrl.set_position(servo, config.SERVO_START_POSITION)
    time.sleep(1.0)


def test_sweep(servo, step=0.05, dt=0.05):
    """
    Sweep the servo continuously over its full range until interrupted
    :param servo: GPIO chip handle
    :param step: Position increment per iteration
    :param dt: Time between two increments, in seconds
    """
    print("Sweeping, press Ctrl+C to stop")
    position = -1.0
    direction = 1.0

    while True:
        servo_ctrl.set_position(servo, position)
        time.sleep(dt)

        position += direction * step
        if position >= 1.0:
            position = 1.0
            direction = -1.0
        elif position <= -1.0:
            position = -1.0
            direction = 1.0


def test_with_leds(servo, leds, duration=20.0):
    """
    Toggle the servo while driving the LEDs at the main loop rate, as main.py does
    :param servo: GPIO chip handle
    :param leds: NeoPixel object
    :param duration: Duration of the test, in seconds
    """
    print("Toggling the servo every 2 s while refreshing the LEDs, press Ctrl+C to stop")
    time_start = time.time()
    position = config.SERVO_START_POSITION
    last_toggle = time_start

    while time.time() - time_start < duration:
        time_now = time.time()

        if time_now - last_toggle >= 2.0:
            position = config.SERVO_END_POSITION if position == config.SERVO_START_POSITION else config.SERVO_START_POSITION
            last_toggle = time_now
            print(f"Position {position}")

        servo_ctrl.set_position(servo, position)

        leds.fill(config.BLUE)
        leds.show()

        time.sleep(config.DT_MAIN)


def test_idle(servo):
    """
    Move to the start position and cut the pulses, to check the launcher still holds undriven
    :param servo: GPIO chip handle
    """
    print(f"Start position: {config.SERVO_START_POSITION}")
    servo_ctrl.set_position(servo, config.SERVO_START_POSITION)
    time.sleep(config.SERVO_TRAVEL_TIME)

    print("Pulses stopped, check that the launcher still holds. Ctrl+C to stop")
    servo_ctrl.stop(servo)

    while True:
        time.sleep(1.0)


def _servo_child(duration):
    """
    Open and drive the servo from inside a child process, as main.py does
    :param duration: Duration of the test, in seconds
    """
    import leds_ctrl

    leds = leds_ctrl.leds_init()
    servo = servo_ctrl.servo_init()
    if servo is None:
        print(f"Child process: could not open servo on GPIO {config.SERVO_PIN}")
        return

    try:
        test_with_leds(servo, leds, duration)
    finally:
        servo_ctrl.set_position(servo, config.SERVO_START_POSITION)
        time.sleep(0.5)  # let the servo travel back before it stops being driven
        servo_ctrl.servo_deinit(servo)


def _rc_child(shared_remote_command, shared_target_speed, shared_wheel_position, shared_knobl_value, shared_knobr_value, restart_event):
    """
    Run the RC reader, importing main inside the child so the parent stays clean of lgpio
    """
    import main

    main.rc_receiver_reading(shared_remote_command, shared_target_speed, shared_wheel_position,
                             shared_knobl_value, shared_knobr_value, restart_event)


def test_like_main(duration=20.0, lazy_import=False):
    """
    Reproduce the process structure of main.py: the servo runs in a child process
    while the RC reader holds gpiochip0 in another one
    :param duration: Duration of the test, in seconds
    :param lazy_import: Import main only inside the children, so the parent never initializes lgpio
    """
    from multiprocessing import Process, Value, Event

    if lazy_import:
        rc_target = _rc_child
    else:
        import main
        rc_target = main.rc_receiver_reading

    restart_event = Event()
    shared_remote_command = Value('i', 0)
    shared_target_speed = Value('d', 0.0)
    shared_wheel_position = Value('d', 0.0)
    shared_knobl_value = Value('d', 0.0)
    shared_knobr_value = Value('d', 0.0)

    p_rc = Process(
        target=rc_target,
        args=(shared_remote_command,
              shared_target_speed,
              shared_wheel_position,
              shared_knobl_value,
              shared_knobr_value,
              restart_event),
    )
    p_servo = Process(target=_servo_child, args=(duration,))

    print("Starting the RC and servo processes")
    p_rc.start()
    p_servo.start()

    try:
        p_servo.join()
    finally:
        restart_event.set()
        p_rc.join(timeout=5)
        if p_rc.is_alive():
            p_rc.terminate()
            p_rc.join()


def test_manual(servo):
    """
    Set the servo position manually from the terminal
    :param servo: GPIO chip handle
    """
    print("Enter a position between -1.0 and 1.0 (empty line or Ctrl+C to quit)")

    while True:
        raw = input("position: ").strip()
        if raw == "":
            return

        try:
            position = float(raw)
        except ValueError:
            print("Not a number")
            continue

        pulse_width = servo_ctrl.position_to_pulse_width(position)
        print(f"Position {position} -> {pulse_width} µs")
        servo_ctrl.set_position(servo, position)


# =========================
# CLI
# =========================
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Choose: 'endpoints', 'release', 'sweep', 'manual', 'idle', 'with_leds', 'like_main', or 'like_main_lazy'")
        sys.exit(0)

    cmd = sys.argv[1]

    # The servo is opened by the child process, not here
    if cmd == "like_main":
        test_like_main()
        sys.exit(0)
    elif cmd == "like_main_lazy":
        test_like_main(lazy_import=True)
        sys.exit(0)

    # Initialize the LEDs before the servo, in the same order as main.py
    leds = None
    if cmd == "with_leds":
        import leds_ctrl
        leds = leds_ctrl.leds_init()

    servo = servo_ctrl.servo_init()
    if servo is None:
        print(f"Could not open servo on GPIO {config.SERVO_PIN}")
        sys.exit(1)

    print(f"Servo on GPIO {config.SERVO_PIN}")

    try:
        if cmd == "endpoints":
            test_endpoints(servo)
        elif cmd == "release":
            test_release(servo)
        elif cmd == "sweep":
            test_sweep(servo)
        elif cmd == "manual":
            test_manual(servo)
        elif cmd == "with_leds":
            test_with_leds(servo, leds)
        elif cmd == "idle":
            test_idle(servo)
        else:
            print(f"Unknown function: {cmd}")
    except (KeyboardInterrupt, EOFError):
        print("\nStopping servo test.")
    finally:
        servo_ctrl.set_position(servo, config.SERVO_START_POSITION)
        time.sleep(0.5)  # let the servo travel back before it stops being driven
        servo_ctrl.servo_deinit(servo)
