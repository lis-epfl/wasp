from picamera2 import Picamera2
import time
from datetime import datetime

# Initialize the camera
picam2 = Picamera2()

# Configure the camera for still image capture
camera_config = picam2.create_still_configuration()
picam2.configure(camera_config)

# Start the camera
picam2.start()
time.sleep(2)  # Give some time for the camera to adjust

save_path = "data"
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"{save_path}/cam_view{timestamp}.jpg"

# Capture and save the image
picam2.capture_file(filename)

print(f"Image saved as {filename}")