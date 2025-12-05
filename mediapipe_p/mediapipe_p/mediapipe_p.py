# keep venv
from . import venv_hack

# ros includes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.time import Time

# Image includes
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
import cv2

# Other includes
from enum import Enum
from typing import Tuple, Union
import math
import numpy as np

# Message includes
from sensor_msgs.msg import Image
from tracking_msg.msg import TrackingImage
from mediapipe_msg.msg import Landmark, PoseStamped

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
model_path = "pose_landmarker.task"
options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.IMAGE)

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

LandMarkEnum = Enum('LandMark', [
    ('nose', 0),
    ('left eye (inner)', 1),
    ('left eye', 2),
    ('left eye (outer)', 3),
    ('right eye (inner)', 4),
    ('right eye', 5),
    ('right eye (outer)', 6),
    ('left ear', 7),
    ('right ear', 8),
    ('mouth (left)', 9),
    ('mouth (right)', 10),
    ('left shoulder', 11),
    ('right shoulder', 12),
    ('left elbow', 13),
    ('right elbow', 14),
    ('left wrist', 15),
    ('right wrist', 16),
    ('left pinky', 17),
    ('right pinky', 18),
    ('left index', 19),
    ('right index', 20),
    ('left thumb', 21),
    ('right thumb',22),
    ('left hip', 23),
    ('right hip', 24),
    ('left knee', 25),
    ('right knee', 26),
    ('left ankle', 27),
    ('right ankle', 28),
    ('left heel', 29),
    ('right heel', 30),
    ('left foot index', 31),
    ('right foot index', 32),
])

class MediaPipe(Node):

    def __init__(self):
        super().__init__('mediapipe')
        self.img_subscriber = self.create_subscription(
            TrackingImage,
            '/tracking/tracked_image',
            self.img_parser,
            10)
        self.img_subscriber
        self.bridge = CvBridge()
        self.debug_publisher = self.create_publisher(Image, '/landmark/debug_image', 10)
        self.debug_original_publisher = self.create_publisher(Image, '/landmark/debug_original_image', 10)
        self.landmark_publisher = self.create_publisher(PoseStamped, '/landmark/normalized_pose', 10)
        self.landmarker = PoseLandmarker.create_from_options(options)

    # From mediapipe drawing_utils
    def _normalized_to_pixel_coordinates(
            self,
            normalized_x: float, normalized_y: float, image_width: int,
            image_height: int) -> Union[None, Tuple[int, int]]:
        """Converts normalized value pair to pixel coordinates."""

        # Checks if the float value is between 0 and 1.
        def is_valid_normalized_value(value: float) -> bool:
            return (value > 0 or math.isclose(0, value)) and (value < 1 or
                                                            math.isclose(1, value))

        if not (is_valid_normalized_value(normalized_x) and
                is_valid_normalized_value(normalized_y)):
            # TODO: Draw coordinates even if it's outside of the image bounds.
            return None
        x_px = min(math.floor(normalized_x * image_width), image_width - 1)
        y_px = min(math.floor(normalized_y * image_height), image_height - 1)
        return x_px, y_px
    
    def _pixel_coordinates_to_full_image_coordinates(self, pixel_x, pixel_y, crop_x1, crop_y1):
        x_full = pixel_x + crop_x1
        y_full = pixel_y + crop_y1
        return x_full, y_full
    
    def _full_image_coordinates_to_normalized(self, pixel_x, pixel_y, full_width, full_height):
        x_norm = pixel_x / full_width
        y_norm = pixel_y / full_height
        return x_norm, y_norm

    def img_parser(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg.frame, "rgb8")
        except CvBridgeError as e:
            self.get_logger().info("Failed: ", str(e))
            return
        # cv_image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        results = self.landmarker.detect(mp_image)
        pose_landmarks_list = results.pose_landmarks
        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            normalized_data = [landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks]
            pose_landmarks_proto.landmark.extend(normalized_data)
            mp_drawing.draw_landmarks(
                image, pose_landmarks_proto, mp_pose.POSE_CONNECTIONS)

        debug_image = self.bridge.cv2_to_imgmsg(image, "rgb8")
        debug_image.header.stamp = self.get_clock().now().to_msg()
        debug_image.header.frame_id = "camera_rgb_frame"
        self.debug_publisher.publish(debug_image)
        for human in results.pose_landmarks:
            new_pose = PoseStamped()
            new_pose.pose.id = msg.id
            new_pose.header.stamp = self.get_clock().now().to_msg()
            new_pose.header.frame_id = "camera_rgb_frame"
            black_image = np.zeros((msg.original_height, msg.original_width, 3), dtype=np.uint8)
            for idx, landmark in enumerate(human):
                new_landmark = Landmark()
                new_landmark.name = LandMarkEnum(idx).name
                im_rows, im_cols = mp_image.height, mp_image.width
                pix_xy = self._normalized_to_pixel_coordinates(landmark.x, landmark.y, im_cols, im_rows)
                pixel_x = None
                pixel_y = None
                if pix_xy is not None:
                    pixel_x, pixel_y = pix_xy
                    x_full, y_full = self._pixel_coordinates_to_full_image_coordinates(pixel_x, pixel_y, msg.x1, msg.y1)
                    norm_x, norm_y = self._full_image_coordinates_to_normalized(x_full, y_full, msg.original_width, msg.original_height)
                    new_landmark.pixel_x = x_full
                    new_landmark.pixel_y = y_full
                    new_landmark.normalized_x = norm_x
                    new_landmark.normalized_y = norm_y
                    cv2.circle(black_image, (int(x_full), int(y_full)), radius=3, color=(0, 255, 0), thickness=-1)
                    new_landmark.is_pixel_valid = True
                else:
                    new_landmark.normalized_x = landmark.x
                    new_landmark.normalized_y = landmark.y
                    new_landmark.is_pixel_valid = False
                new_landmark.normalized_z = landmark.z
                new_landmark.visibility = landmark.visibility
                new_landmark.presence = landmark.presence
                new_pose.pose.landmarks[idx] = new_landmark
            debug_image = self.bridge.cv2_to_imgmsg(black_image, "rgb8")
            debug_image.header.stamp = self.get_clock().now().to_msg()
            debug_image.header.frame_id = "camera_rgb_frame"
            self.debug_original_publisher.publish(debug_image)
            self.landmark_publisher.publish(new_pose)


def main(args=None):
    try:
        with rclpy.init(args=args):
            mediapipe = MediaPipe()

            rclpy.spin(mediapipe)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()