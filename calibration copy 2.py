import cv2
import numpy as np
import openni
from openni import openni2
from time import sleep

class Resolution:
    def __init__(self, x, y, fps):
        self.x = x
        self.y = y
        self.fps = fps
get_color_frame = True
get_depth_frame = True

# Initialize OpenNI
openni2.initialize()

# Open the device
dev = openni2.Device.open_any()

# Define the resolution and frame rate
color_res = Resolution(x=640, y=480, fps=30)
depth_res = Resolution(x=640, y=480, fps=30)

# Function to capture a single frame from a stream
def capture_single_frame(stream, pixel_format, resolution):
    stream.set_video_mode(openni2.VideoMode(pixelFormat=pixel_format, resolutionX=resolution.x, resolutionY=resolution.y, fps=resolution.fps))
    stream.start()
    frame = stream.read_frame()
    stream.stop()
    return frame

# Capture a single color frame
if get_color_frame:
    color_stream = dev.create_color_stream()
    color_frame = capture_single_frame(color_stream, openni2.PIXEL_FORMAT_RGB888, color_res)
    color_buffer = color_frame.get_buffer_as_triplet()
    color_data_unshaped = np.frombuffer(color_buffer, dtype=np.uint8)
    color_data = color_data_unshaped.reshape(color_res.y, color_res.x, 3)
    color_data = cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR)
    cv2.imshow("RGB", color_data)

# Capture a single depth frame
if get_depth_frame:
    depth_stream = dev.create_depth_stream()
    depth_frame = capture_single_frame(depth_stream, openni2.PIXEL_FORMAT_DEPTH_1_MM, depth_res)
    depth_buffer = depth_frame.get_buffer_as_uint16()
    depth_data_unshaped = np.frombuffer(depth_buffer, dtype=np.uint16)
    depth_data = depth_data_unshaped.reshape(depth_res.y, depth_res.x)
    cv2.imshow("Depth", depth_data)

# Wait for a key press to close the windows
cv2.waitKey(0)

# Close OpenNI
openni2.unload()
cv2.destroyAllWindows()