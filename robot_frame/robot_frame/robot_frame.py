import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from rclpy.time import Time

from follow_msg.msg import Target
from mediapipe_msg.msg import PoseStamped
from sensor_msgs.msg import LaserScan, CameraInfo, Image

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
        self.camera_depth_info_subscription = self.create_subscription(
            CameraInfo,
            '/astra/depth/camera_info',
            self.get_depth_camera_instrinsics,
            10
            )
        self.camera_depth_subscription = self.create_subscription(
            Image,
            '/astra/depth/image_raw',
            self.get_depth_camera_image,
            10
            )
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_processor,
            10)
            
        self.target_publisher = self.create_publisher(Target, '/follow/target', 10)
        
        self.pose_subscription
        self.camera_info_subscription
        self.lidar_subscription
        self.lidar_scan = None
        self.camera_info = None
        self.depth_camera_info = None
        self.depth_camera = None
        self.bridge = CvBridge()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def pose_processor(self, msg):
        # no lidar nothing to do
        if not self.camera_info:
            print("Missing Camera info")
        if not self.depth_camera:
            print("Missing Depth Camera")
        if not self.depth_camera_info:
            print("Missing Depth Camera info")
        if not self.camera_info or not self.depth_camera or not self.depth_camera_info:
        # if not self.lidar_scan or not self.camera_info or not self.depth_camera or not self.depth_camera_info:
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(self.depth_camera)
        except CvBridgeError as e:
            self.get_logger().info("Failed: ", str(e))
            return
        angle_rads = []
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        name_to_depth = {}

        dci = self.depth_camera.header.frame_id
        t = self.tfBuffer.lookup_transform("base_link", dci, Time())
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        translation = t.transform.translation
        T = np.array([translation.x, translation.y, translation.z])
        print(depth_image.shape)
        tolerance_x = 25
        tolerance_y = 25
        depth_tolerance = 1e-3
        height, width = depth_image.shape
        for landmark in msg.pose.landmarks:
            if not landmark.is_pixel_valid:
                continue
            # take out edges
            if landmark.pixel_x <= tolerance_x or landmark.pixel_x >= width - tolerance_x:
                continue
            if landmark.pixel_y <= tolerance_y or landmark.pixel_y >= height - tolerance_y:
                continue
            angle_from_center_rad = np.arctan2((landmark.pixel_x - cx), fx)
            angle_rads.append(angle_from_center_rad)
            name = landmark.name
            # Assume same?
            depth_point = depth_image[landmark.pixel_y][landmark.pixel_x] / 1000.
            if depth_point < depth_tolerance:
                continue
            real_x = (landmark.pixel_x - cx) * depth_point / fx
            real_y = (landmark.pixel_y - cy) * depth_point / fy
            p_cam = np.array([real_x, real_y, depth_point])
            p_base = R @ p_cam + T
            name_to_depth[name] = p_base[2]
            # name_to_depth[name] = depth_point
        angle_rads = np.array(angle_rads)
        mean_angle_rad = np.mean(angle_rads)
        # positve clockwise
        print(f"Mean angle: {mean_angle_rad}")
        depth_m = np.array(list(name_to_depth.values()))
        mean_depth_m = np.mean(depth_m)
        print(f"Mean depth: {mean_depth_m}")
        print("\n")
        for name, value in sorted(name_to_depth.items(), key=lambda item: item[1]):
            print(f"{name} is at {value}m")
        print("-----")
        if np.isnan(mean_angle_rad) or np.isnan(mean_depth_m):
            return
        new_target = Target()
        new_target.header.stamp = self.get_clock().now().to_msg()
        new_target.header.frame_id = "base_link"
        new_target.angle_from_center_rad = mean_angle_rad
        new_target.depth_m = mean_depth_m
        self.target_publisher.publish(new_target)
        # tf2_kdl pykdl
        # transform_stamped = self.tf_buffer.lookup_transform("base_link", msg.header.frame_id, msg.header.stamp);
        # Will want to multiply rotation matrix with my angle to get angle in correct frame of reference
        # For now assume same angle
        delta = 5
        # mid = self.lidar_scan.ranges[0: delta] + self.lidar_scan.ranges[-delta:]
        # print(mid)


    def get_depth_camera_instrinsics(self, msg):
        self.depth_camera_info = msg

    def get_depth_camera_image(self, msg):
        self.depth_camera = msg

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