import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


from follow_msg.msg import Target
from geometry_msgs.msg import Twist
import time

import numpy as np

class LocalMovement(Node):

    def __init__(self):
        super().__init__('local_movement')
        self.target_subscription = self.create_subscription(
            Target,
            '/follow/target',
            self.process_target,
            10)
        self.target_subscription

        self.move_command_publisher = self.create_publisher(Twist, '/commands/velocity', 10)

        self.last_msg_time = time.time()
        self.timer = self.create_timer(0.1, self.check_timeout)


    def process_target(self, msg):
        angle_from_center_rad = msg.angle_from_center_rad
        depth_m = msg.depth_m
        print(f"Angle: {angle_from_center_rad}")
        print(f"Depth: {depth_m}")
        self.last_msg_time = time.time()
        tolerance_depth_m = 0.9
        tolerance_angle_rad = 0.01

        new_move_command = Twist()
        new_move_command.linear.x = 0. if depth_m < tolerance_depth_m else depth_m * 0.05
        new_move_command.angular.z = 0. if abs(angle_from_center_rad) < tolerance_angle_rad else -angle_from_center_rad * 0.9
        self.move_command_publisher.publish(new_move_command)

    def check_timeout(self):
        timeout_s = 2.0
        if time.time() - self.last_msg_time > timeout_s:
            zero_velocity = Twist()
            self.move_command_publisher.publish(zero_velocity)
            self.get_logger().info("No command received in {timeout_s} seconds")



def main(args=None):
    try:
        with rclpy.init(args=args):
            local_movement = LocalMovement()

            rclpy.spin(local_movement)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
