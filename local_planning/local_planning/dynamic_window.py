# ros includes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# Other includes
import numpy as np
import math

# Message includes
from follow_msg.msg import Target
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time


class DynamicWindowPlanner(Node):

    def __init__(self):
        super().__init__('dynamic_window_planner')
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.set_scan,
            10
        )
        self.global_command_subscriber = self.create_subscription(
            Target,
            '/command/target',
            self.set_target,
            10)
        self.scan_subscriber
        self.global_command_subscriber
        self.local_publisher = self.create_publisher(Twist, '/commands/velocity', 10)
        self.last_scan = None
        self.last_msg_time = time.time()
        self.timer = self.create_timer(0.1, self.update)
        self.goal_angle_rad = 0.0
        self.goal_depth_m = None

        self.v_min, self.v_max = 0.0, 0.11
        self.w_min, self.w_max = -0.5, 0.5

        self.robot_radius = 0.18
        self.dt = 0.1
        self.horizon = 0.8

    def update(self):
        timeout_s = 2.0
        timeout_delta = time.time() - self.last_msg_time
        if timeout_delta > timeout_s:
            self.goal_angle_rad = 0.0
            self.goal_depth_m = None
            zero_velocity = Twist()
            self.local_publisher.publish(zero_velocity)
            self.get_logger().info(f"No command received in {timeout_delta} seconds")
            return
        
        # No lidar scan topic
        if self.last_scan is None:
            return
        
        # At target stop or target doesn't make sense
        if self.goal_depth_m is None or self.goal_depth_m <= 1.0:
            self.local_publisher.publish(Twist())
            return
        self.get_logger().info(f"Depth {self.goal_depth_m}")
        self.get_logger().info(f"Angle {self.goal_angle_rad}")
        
        v_samples = np.linspace(self.v_min, self.v_max, 5)
        w_samples = np.linspace(self.w_min, self.w_max, 7)

        best_score = -1e9
        best_v, best_w = 0.0, 0.0

        for v in v_samples:
            for w in w_samples:
                if not self.trajectory_safe(v, w):
                    continue

                desired_w = self.goal_angle_rad / self.horizon
                score_heading = -abs(w - desired_w)

                score_speed = v

                score = score_speed + 0.4 * score_heading

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.get_logger().info(f"Cmd {cmd}")
        self.local_publisher.publish(cmd)

    def trajectory_safe(self, v, w):
        steps = int(self.horizon / self.dt)
        x, y, yaw = 0., 0., 0.

        angles = np.arange(len(self.last_scan.ranges)) * self.last_scan.angle_increment + self.last_scan.angle_min
        ranges = np.array(self.last_scan.ranges)
        ranges[ranges < 0.30] = float('inf')

        for _ in range(steps):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt

            px = ranges * np.cos(angles)
            py = ranges * np.sin(angles)

            d = np.sqrt((px - x)**2 + (py - y)**2)
            if np.any(d < self.robot_radius):
                return False
        return True

    def set_scan(self, msg):
        self.last_scan = msg

    def set_target(self, msg):
        self.last_msg_time = time.time()
        # inverse goal direction angle.
        self.goal_angle_rad = -msg.angle_from_center_rad
        self.goal_depth_m = msg.depth_m



def main(args=None):
    try:
        with rclpy.init(args=args):
            dynamic_window_planner = DynamicWindowPlanner()

            rclpy.spin(dynamic_window_planner)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()