from picamera2 import Picamera2

import config


def camera_init():
    """
    Initialize the camera and configure it for video recording.
    """
    # Initialize the camera
    picam2 = Picamera2()

    # Configure the camera for video recording
    video_config = picam2.create_video_configuration()
    picam2.configure(video_config)
    picam2.set_controls({"FrameRate": config.FRAME_RATE})

    return picam2