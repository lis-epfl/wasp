import config
import motor_ctrl

def update(last_state, remote_command, in_calibration_mode):
    """
    Update the state of the cart based on the button pressed.

    GO_TAKE_OFF enters TRACKING from the manual states even though take off itself is only ever
    entered from TRACKING below. With the launch switch already on launch, the RC process goes
    from a manual command to GO_TAKE_OFF within a single RC period (DT_RC), so this loop (DT_MAIN,
    ten times slower) never observes the GO_TRACKING in between and would stay in STOP forever.
    """
    state = last_state

    if last_state == config.STATE["STOP"]:
        if remote_command in (config.REMOTE_COMMAND["GO_TRACKING"], config.REMOTE_COMMAND["GO_TAKE_OFF"]) and not in_calibration_mode:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["BACKWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["FORWARD"]
        else:
            state = config.STATE["STOP"]

    elif last_state == config.STATE["FORWARD"]:
        if remote_command in (config.REMOTE_COMMAND["GO_TRACKING"], config.REMOTE_COMMAND["GO_TAKE_OFF"]) and not in_calibration_mode:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["FORWARD"]
        else:
            state = config.STATE["FORWARD"]

    elif last_state == config.STATE["BACKWARD"]:
        if remote_command in (config.REMOTE_COMMAND["GO_TRACKING"], config.REMOTE_COMMAND["GO_TAKE_OFF"]) and not in_calibration_mode:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["BACKWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["STOP"]
        else:
            state = config.STATE["BACKWARD"]

    elif last_state == config.STATE["TRACKING"]:
        if in_calibration_mode:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] or remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TAKE_OFF"]:
            state = config.STATE["TAKE_OFF"]
        else:
            state = config.STATE["TRACKING"]
    
    elif last_state == config.STATE["TAKE_OFF"]:
        if in_calibration_mode:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] or remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"]:
            state = config.STATE["TRACKING"]
        else:
            state = config.STATE["TAKE_OFF"]
    else:
        state = config.STATE["STOP"]

    # Always allow STOP command (common to all cases)
    if remote_command == config.REMOTE_COMMAND["GO_STOP"]:
        state = config.STATE["STOP"]
    
    return state