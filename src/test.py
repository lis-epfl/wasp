import config
import motor_ctrl

def update(last_state, remote_command, obstacle_forward, obstacle_backward, current_linear_position, current_angular_velocity):
    """
    Update the state of the cart based on the button pressed, obstacle detection, and position.
    Adapted for position control: safety relies on anticipating deceleration distance.
    """
    state = last_state

    # Estimate how far we need to stop safely from current speed
    deceleration_distance = motor_ctrl.compute_linear_position((current_angular_velocity ** 2) / (2 * config.MAX_ACCELERATION))

    # Check if we're approaching physical limits
    reached_end = current_linear_position >= (config.ZIPLINE_LENGTH - config.SECURITY_FACTOR * deceleration_distance)
    reached_start = current_linear_position <= (config.ZIPLINE_START + config.SECURITY_FACTOR * deceleration_distance)

    if last_state == config.STATE["STOP"]:
        if remote_command == config.REMOTE_COMMAND["GO_TRACKING"] and not (obstacle_forward or obstacle_backward or reached_end or reached_start):
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] and not obstacle_backward and not reached_start:
            state = config.STATE["BACKWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not obstacle_forward and not reached_end:
            state = config.STATE["FORWARD"]

    elif last_state == config.STATE["FORWARD"]:
        if obstacle_forward or reached_end:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"] and not reached_end:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not reached_end:
            state = config.STATE["FORWARD"]

    elif last_state == config.STATE["BACKWARD"]:
        if obstacle_backward or reached_start:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"] and not reached_start:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] and not reached_start:
            state = config.STATE["BACKWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["STOP"]

    elif last_state == config.STATE["TRACKING"]:
        if obstacle_forward or obstacle_backward or reached_end or reached_start:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] or remote_command == config.REMOTE_COMMAND["GO_FORWARD"]:
            state = config.STATE["STOP"]

    # Always allow STOP command
    if remote_command == config.REMOTE_COMMAND["GO_STOP"]:
        state = config.STATE["STOP"]

    return state