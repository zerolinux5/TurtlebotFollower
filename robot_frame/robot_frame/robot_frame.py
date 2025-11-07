import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tf2_ros

from mediapipe_msg.msg import PoseStamped
from sensor_msgs.msg import LaserScan, CameraInfo

import numpy as np


class RobotFrameTransform(Node):

    def __init__(self):
        super().__init__('robot_frame_transform')
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/landmark/normalized_pose',
            self.pose_processor,
            10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/astra/color/camera_info',
            self.get_camera_instrinsics,
            10
            )
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_processor,
            10)
        
        self.pose_subscription
        self.camera_info_subscription
        self.lidar_subscription
        self.lidar_scan = None
        self.camera_info = None
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def pose_processor(self, msg):
        # no lidar nothing to do
        if not self.camera_info:
            return
        angle_rads = []
        fx = self.camera_info.k[0]
        cx = self.camera_info.k[2]
        for landmark in msg.pose.landmarks:
            if not landmark.is_pixel_valid:
                continue
            angle_from_center = np.arctan2((landmark.pixel_x - cx), fx)
            angle_rads.append(angle_from_center)
        angle_rads = np.array(angle_rads)
        # positve clockwise
        print(f"Mean: {np.mean(angle_rads)}")
        print("-----")
        # tf2_kdl pykdl
        # transform_stamped = self.tf_buffer.lookup_transform("base_link", msg.header.frame_id, msg.header.stamp);
        # Will want to multiply rotation matrix with my angle to get angle in correct frame of reference
        # For now assume same angle



    def lidar_processor(self, msg):
        self.lidar_scan = msg

    def get_camera_instrinsics(self, msg):
        # no lidar nothing to do
        self.camera_info = msg


def main(args=None):
    try:
        with rclpy.init(args=args):
            robot_frame_transform = RobotFrameTransform()

            rclpy.spin(robot_frame_transform)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()