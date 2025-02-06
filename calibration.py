import csv
import json
import cv2
import numpy as np
import openni
from openni import openni2
from time import sleep
import glob

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

n_frames = 10

# # Capture and save multiple frames
# color_frames = []
# depth_frames = []

# for i in range(n_frames):  # Capture 10 frames
#     if get_color_frame:
#         color_stream = dev.create_color_stream()
#         color_frame = capture_single_frame(color_stream, openni2.PIXEL_FORMAT_RGB888, color_res)
#         color_buffer = color_frame.get_buffer_as_triplet()
#         color_data_unshaped = np.frombuffer(color_buffer, dtype=np.uint8)
#         color_data = color_data_unshaped.reshape(color_res.y, color_res.x, 3)
#         color_data = cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR)
#         color_frames.append(color_data)
#         cv2.imshow("RGB", color_data)
#         cv2.imwrite(f'color_frame_{i}.png', color_data)

#     if get_depth_frame:
#         depth_stream = dev.create_depth_stream()
#         depth_frame = capture_single_frame(depth_stream, openni2.PIXEL_FORMAT_DEPTH_1_MM, depth_res)
#         depth_buffer = depth_frame.get_buffer_as_uint16()
#         depth_data_unshaped = np.frombuffer(depth_buffer, dtype=np.uint16)
#         depth_data = depth_data_unshaped.reshape(depth_res.y, depth_res.x)
#         depth_frames.append(depth_data)
#         cv2.imshow("Depth", depth_data)
#         cv2.imwrite(f'depth_frame_{i}.png', depth_data)

#     cv2.waitKey(500)  # Wait for 500ms between frames

# # Close OpenNI
# openni2.unload()
# cv2.destroyAllWindows()

# Load the saved frames for calibration
color_images = [cv2.imread(file) for file in glob.glob('color_frame_*.png')]

# Check if images are loaded correctly
if not color_images:
    print("No images found for calibration.")
    exit()

# Define the chessboard size
chessboard_size = (10, 7)

# Prepare object points
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane

# Find the chessboard corners in the images
for i, img in enumerate(color_images):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        
        print(f'Image {i}: Chessboard found.')

        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard', img)
        cv2.imwrite(f'chess_board_frame_{i}.png', img)
        cv2.waitKey(500)
    else:
        print(f'Image {i}: NO, Chessboard not found!')

cv2.destroyAllWindows()

# Check if enough points were found
if not objpoints or not imgpoints:
    print("Not enough points found for calibration.")
    exit()

# Perform camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


# Save the camera matrix (intrinsics) to a JSON file
intrinsics = {
    'camera_matrix': mtx.tolist(),
    'distortion_coefficients': dist.tolist()
}
with open('camera_intrinsics.json', 'w') as file:
    json.dump(intrinsics, file, indent=4)

# Save the rotation and translation vectors (extrinsics) to a JSON file
extrinsics = {
    'rotation_vectors': [rvec.tolist() for rvec in rvecs],
    'translation_vectors': [tvec.tolist() for tvec in tvecs]
}
with open('camera_extrinsics.json', 'w') as file:
    json.dump(extrinsics, file, indent=4)
