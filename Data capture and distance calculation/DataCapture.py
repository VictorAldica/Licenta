import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time

class FrameProcessor(threading.Thread):
    def __init__(self):
        super(FrameProcessor, self).__init__()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.depth_scale = None
        self.color_image = None
        self.depth_image = None
        self.lock = threading.Lock()
        self._stop_event = threading.Event()

        self.width = 640

        self.config.enable_stream(rs.stream.depth, self.width, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.width, 480, rs.format.bgr8, 30)

        self.profile = self.pipeline.start(self.config)

        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        self.align = rs.align(rs.stream.color)

    def run(self):
        try:
            while not self._stop_event.is_set():
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                with self.lock:
                    self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
                    self.color_image = np.asanyarray(color_frame.get_data())
        finally:
            self.pipeline.stop()

    def stop(self):
        self._stop_event.set()

    def get_frames(self):
        with self.lock:
            if self.color_image is not None and self.depth_image is not None:
                return self.color_image.copy(), self.depth_image.copy()
            else:
                return None, None

    def calculate_distance(self, depth_image):
        depth_in_meters = depth_image * self.depth_scale

        masked_depth = np.ma.masked_less_equal(depth_in_meters, 0.0001)

        min_distance = np.nanmin(masked_depth)
        min_distance_coordinates = np.unravel_index(np.nanargmin(masked_depth), masked_depth.shape)

        max_distance = np.nanmax(masked_depth)
        max_distance_coordinates = np.unravel_index(np.nanargmax(masked_depth), masked_depth.shape)

        section = self.get_section(min_distance_coordinates)

        return min_distance, min_distance_coordinates, max_distance, max_distance_coordinates, section

    def get_section(self, coordinates):
        _, y = coordinates
        if y < self.width/4:
            return 'Left'
        elif y < self.width/2:
            return 'Middle Left'
        elif y < self.width*3/4:
            return 'Middle Right'
        else:
            return 'Right'

frame_processor = FrameProcessor()
frame_processor.start()

try:
    while True:
        color_image, depth_image = frame_processor.get_frames()

        if color_image is None or depth_image is None:
            continue

        min_distance, min_distance_coordinates, max_distance, max_distance_coordinates, section = frame_processor.calculate_distance(depth_image)

        print(f"Min distance: {min_distance} m in section {section}")
        print(f"Max distance: {max_distance} m")

        cv2.circle(color_image, (min_distance_coordinates[1], min_distance_coordinates[0]), 5, (0, 255, 0), -1)
        cv2.putText(color_image, f"{min_distance} m", (min_distance_coordinates[1], min_distance_coordinates[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.circle(color_image, (max_distance_coordinates[1], max_distance_coordinates[0]), 5, (0, 0, 255), -1)
        cv2.putText(color_image, f"{max_distance} m", (max_distance_coordinates[1], max_distance_coordinates[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow('Color Image', color_image)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        elif key == ord('s'):
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            cv2.imwrite(f"color_image_{timestamp}.png", color_image)
            cv2.imwrite(f"depth_image_{timestamp}.png", depth_image)

finally:
    frame_processor.stop()
