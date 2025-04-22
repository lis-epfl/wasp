import config
import motor_ctrl


def update(last_state, remote_command, obstacle_forward, obstacle_backward, current_linear_position, current_angular_velocity):
    """
    Update the state of the cart based on the button pressed, obstacle detection, and position.
    """
    state = last_state

    # Check zipline limits
    reached_end = current_linear_position >= (config.ZIPLINE_LENGTH - motor_ctrl.compute_linear_position((current_angular_velocity ** 2) / (2 * config.MOTOR_ACCELERATION)))
    reached_start = current_linear_position <=  motor_ctrl.compute_linear_position((current_angular_velocity ** 2) / (2 * config.MOTOR_ACCELERATION))

    if last_state == config.STATE["STOP"]:
        if remote_command == config.REMOTE_COMMAND["GO_TRACKING"] and not (obstacle_forward or obstacle_backward):
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] and not obstacle_backward and not reached_start:
            state = config.STATE["BACKWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not obstacle_forward and not reached_end:
            state = config.STATE["FORWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_STOP"]:
            state = config.STATE["STOP"]

    elif last_state == config.STATE["FORWARD"]:
        if obstacle_forward or reached_end:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"]:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"]:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not reached_end:
            state = config.STATE["FORWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_STOP"]:
            state = config.STATE["STOP"]

    elif last_state == config.STATE["BACKWARD"]:
        if obstacle_backward or reached_start:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"]:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] and not reached_start:
            state = config.STATE["BACKWARD"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not reached_end and not obstacle_forward:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_STOP"]:
            state = config.STATE["STOP"]

    elif last_state == config.STATE["TRACKING"]:
        if obstacle_forward or obstacle_backward or reached_end:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_TRACKING"]:
            state = config.STATE["TRACKING"]
        elif remote_command == config.REMOTE_COMMAND["GO_BACKWARD"] and not reached_start:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_FORWARD"] and not reached_end:
            state = config.STATE["STOP"]
        elif remote_command == config.REMOTE_COMMAND["GO_STOP"]:
            state = config.STATE["STOP"]

    return state