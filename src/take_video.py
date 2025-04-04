from picamera2 import Picamera2
from picamera2.encoders import H264Encoder, Quality
from picamera2.outputs import FfmpegOutput
import time
from datetime import datetime

# Initialize the camera
picam2 = Picamera2()

# Configure the camera for video recording
video_config = picam2.create_video_configuration()
picam2.configure(video_config)
picam2.set_controls({"FrameRate": 60})

# Generate a unique filename with timestamp
save_path = "data"
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"{save_path}/video_{timestamp}.mp4"

# Initialize the encoder and output
encoder = H264Encoder()
output = FfmpegOutput(filename)

try:
    print(f"Recording started: {filename}")
    picam2.start_recording(encoder, output, quality=Quality.HIGH)

    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping recording...")
    picam2.stop_recording()
    print(f"Video saved as {filename}")