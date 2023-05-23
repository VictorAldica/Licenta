import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()


config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)


profile = pipeline.start(config)


depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

align_to = rs.stream.color
align = rs.align(align_to)

def calculate_distance(depth_image, depth_scale):
    depth_in_meters = depth_image * depth_scale

    min_distance = np.min(depth_in_meters)
    min_distance_coordinates = np.unravel_index(np.argmin(depth_in_meters), depth_in_meters.shape)

    return min_distance, min_distance_coordinates

try:
    while True:
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())

        min_distance, min_distance_coordinates = calculate_distance(depth_image, depth_scale)

        send_to_arduino(min_distance)

        cv2.imshow('Depth Image', depth_image)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop()
