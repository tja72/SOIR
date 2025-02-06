import cv2
import numpy as np
import openni
from openni import openni2
from time import sleep

#cv2.setLogLevel(6)

# Initialize OpenNI
openni2.initialize()

get_color_frame = True
get_depth_frame = True
class Resolution:
    def __init__(self, x, y, fps):
        self.x = x
        self.y = y
        self.fps = fps
# Open the device
dev = openni2.Device.open_any()
dev.set_depth_color_sync_enabled(True)
# color_res = {
#     "x": 1280,
#     "y": 1024,
#     "fps": 30
# }
# depth_res = {
#     "x": 640,
#     "y": 480,
#     "fps": 30
# }
#color_res = object(x=1280, y=1024, fps=30)
#depth_res = object(x=640, y=480, fps=30)
color_res = Resolution(x=640, y=480, fps=30)
depth_res = Resolution(x=320, y=240, fps=30)



# Create color and depth streams
if get_color_frame:
    color_stream = dev.create_color_stream()
    color_stream.set_video_mode(openni2.VideoMode(pixelFormat=openni2.PIXEL_FORMAT_RGB888, resolutionX=color_res.x, resolutionY=color_res.y, fps=color_res.fps))
    color_stream.start()
    cv2.namedWindow("RGB")
if get_depth_frame:
    depth_stream = dev.create_depth_stream()
    depth_stream.set_video_mode(openni2.VideoMode(pixelFormat=openni2.PIXEL_FORMAT_DEPTH_100_UM, resolutionX=depth_res.x, resolutionY=depth_res.y, fps=depth_res.fps))
    depth_stream.start()
    cv2.namedWindow("Depth")



while True:
    try:
        if get_color_frame:
            # Read a new frame from the color stream
            color_frame = color_stream.read_frame()
            if not color_frame:
                print("No color frame")
            color_buffer =  color_frame.get_buffer_as_triplet()
            color_data_unshaped = np.frombuffer(color_buffer, dtype=np.uint8)
            color_data = color_data_unshaped.reshape(color_res.y, color_res.x, 3)
            color_data = cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR)

            cv2.imshow("RGB", color_data)
            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if get_depth_frame:
            # Read a new frame from the depth stream
            depth_frame = depth_stream.read_frame()
            if not depth_frame:
                print("No depth frame")
            depth_buffer = depth_frame.get_buffer_as_uint16()
            depth_data_unshaped = np.frombuffer(depth_buffer, dtype=np.uint16)
            depth_data = depth_data_unshaped.reshape(depth_res.y, depth_res.x)

            print("view image")
            # Display the frames
            cv2.imshow("Depth", depth_data)

            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt: 
        break
    except Exception as e:
        print(e)
        print("Skipped frame")


# Stop the streams
color_stream.stop()
depth_stream.stop()

# Close OpenNI
openni2.unload()
cv2.destroyAllWindows()