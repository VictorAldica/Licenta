import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time
from collections import deque

class FrameProcessor(threading.Thread):
    def __init__(self, buffer_length=10, pit_threshold=1.5):
        super(FrameProcessor, self).__init__()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.depth_scale = None
        self.color_image = None
        self.depth_image = None
        self.lock = threading.Lock()
        self._stop_event = threading.Event()

        self.width = 848
        self.height = 480
        self.buffer_length = buffer_length
        self.pit_threshold = pit_threshold
        self.max_distance_history = deque(maxlen=buffer_length)

        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)

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
            return (
                self.color_image.copy() if self.color_image is not None else None,
                self.depth_image.copy() if self.depth_image is not None else None
            )

    def calculate_distance(self, depth_image):
        depth_in_meters = depth_image * self.depth_scale

        masked_depth = np.ma.masked_less_equal(depth_in_meters, 0.0001)

        min_distance = np.nanmin(masked_depth)
        min_distance_coordinates = np.unravel_index(np.nanargmin(masked_depth), masked_depth.shape)

        max_distance = np.nanmax(masked_depth)
        max_distance_coordinates = np.unravel_index(np.nanargmax(masked_depth), masked_depth.shape)

        section = self.get_section(min_distance_coordinates)

        self.max_distance_history.append(max_distance)

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

    def check_for_pit(self, max_distance):
        if len(self.max_distance_history) == self.buffer_length:
            avg_distance = np.mean(self.max_distance_history)
            if max_distance > self.pit_threshold * avg_distance:
                return True
        return False

if __name__ == "__main__":
    frame_processor = FrameProcessor()
    frame_processor.start()

    try:
        while True:
            color_image, depth_image = frame_processor.get_frames()

            if color_image is None or depth_image is None:
                continue

            min_distance, min_distance_coordinates, max_distance, max_distance_coordinates, section = frame_processor.calculate_distance(depth_image)

            print(f"Min distance: {min_distance:.4f} m in section {section}")
            print(f"Max distance: {max_distance:.4f} m")

            if frame_processor.check_for_pit(max_distance):
                print("Pit detected")

            cv2.circle(color_image, (min_distance_coordinates[1], min_distance_coordinates[0]), 5, (0, 255, 0), -1)
            cv2.putText(color_image, f"{min_distance:.4f} m", (min_distance_coordinates[1], min_distance_coordinates[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.circle(color_image, (max_distance_coordinates[1], max_distance_coordinates[0]), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"{max_distance:.4f} m", (max_distance_coordinates[1], max_distance_coordinates[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            # Display section in the top left corner
            cv2.putText(color_image, section, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2, cv2.LINE_AA)

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
