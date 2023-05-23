import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

def calculate_distance(depth_image, x, y):
    return depth_image[y, x] * depth_scale

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        distance = calculate_distance(depth_image, depth_image.shape[1]//2, depth_image.shape[0]//2)
        
        send_to_arduino(distance)

finally:
    pipeline.stop()
