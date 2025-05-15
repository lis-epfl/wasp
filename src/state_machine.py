import config
import motor_ctrl

def update(last_state, remote_command, obstacle_forward, obstacle_backward, linear_position, angular_velocity, decelerating_to_full_stop, calibration_mode, zipline_length):
    """
    Update the state of the cart based on the button pressed, obstacle detection, and position.
    Adapted for position control: safety relies on anticipating deceleration distance.
    """
    state = last_state

    # Estimate how far we need to stop safely from current speed
    deceleration_distance = motor_ctrl.compute_linear_position((angular_velocity ** 2) / (2 * config.MAX_ACCELERATION)) + config.DECELERATION_OFFSET

    # Check if we're approaching physical limits
    if calibration_mode:
        reached_end = linear_position >= (config.ZIPLINE_LENGTH_CALIB - deceleration_distance)
        reached_start = linear_position <= (config.ZIPLINE_START_CALIB + deceleration_distance)
    else:
        reached_end = linear_position >= (zipline_length - deceleration_distance)
        reached_start = linear_position <= deceleration_distance
    
    if not decelerating_to_full_stop:
        if last_state == config.STATE["STOP"]:
            if remote_command == config.REMOTE_COMMAND["GO_TRACKING"] and not (obstacle_forward or obstacle_backward or reached_end or reached_start or calibration_mode):
                state = config.STATE["TRACKING"]
            elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] and not obstacle_backward and not reached_start:
                state = config.STATE["BACKWARD"]
            elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not obstacle_forward and not reached_end:
                state = config.STATE["FORWARD"]
            else:
                state = config.STATE["STOP"]

        elif last_state == config.STATE["FORWARD"]:
            if obstacle_forward or reached_end:
                state = config.STATE["STOP"]
            elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"] and not (reached_end or calibration_mode):
                state = config.STATE["TRACKING"]
            elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
                state = config.STATE["STOP"]
            elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not reached_end:
                state = config.STATE["FORWARD"]
            else:
                state = config.STATE["FORWARD"]

        elif last_state == config.STATE["BACKWARD"]:
            if obstacle_backward or reached_start:
                state = config.STATE["STOP"]
            elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"] and not (reached_start or calibration_mode):
                state = config.STATE["TRACKING"]
            elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] and not reached_start:
                state = config.STATE["BACKWARD"]
            elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
                state = config.STATE["STOP"]
            else:
                state = config.STATE["BACKWARD"]

        elif last_state == config.STATE["TRACKING"]:
            if obstacle_forward or obstacle_backward or reached_end or reached_start or calibration_mode:
                state = config.STATE["STOP"]
            elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] or remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
                state = config.STATE["STOP"]
            else:
                state = config.STATE["TRACKING"]
        else:
            state = config.STATE["STOP"]

    # Always allow STOP command (common to all cases)
    if remote_command == config.REMOTE_COMMAND["GO_STOP"]:
        state = config.STATE["STOP"]
    
    return state, reached_end, reached_start