# ros includes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# Other includes
import numpy as np
import math

# Message includes
from visualization_msgs.msg import Marker
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
        self.debug_angle_marker = self.create_publisher(Marker, '/debug/local_angle_arrow', 10)
        self.last_scan = None
        self.last_msg_time = time.time()
        self.timer = self.create_timer(0.1, self.update)
        self.goal_angle_rad = 0.0
        self.goal_depth_m = None

        self.v_min, self.v_max = 0.05, 0.11
        self.w_min, self.w_max = -0.35, 0.35

        self.robot_radius = 0.29
        self.dt = 0.1
        self.horizon = 1.0

    def update(self):
        timeout_s = 5.0
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
        if self.goal_depth_m is None or self.goal_depth_m < 1.1:
            self.local_publisher.publish(Twist())
            return
        
        v_samples = np.linspace(self.v_min, self.v_max, 20)
        w_samples = np.linspace(self.w_min, self.w_max, 30)

        best_score = -1e9
        best_v, best_w = 0.0, 0.0
        # self.get_logger().info(f"Goal: {np.degrees(self.goal_angle_rad)}")
        # angles = self.last_scan.angle_min + np.arange(len(self.last_scan.ranges)) * self.last_scan.angle_increment
        # ranges = np.array(self.last_scan.ranges)
        # for r, a in zip(ranges, angles):
        #     self.get_logger().info(f"AR: {np.degrees(a)} : {r}")
        for v in v_samples:
            for w in w_samples:
                obstacle_cost = self.get_distance_cost(v, w)
                # self.get_logger().info(f"VWO: {v} : {w} | {obstacle_cost}")
                # if not self.trajectory_safe(v, w):
                #     self.get_logger.info(f"{v} : {w} crashed")
                #     obstacle_cost = 100.0

                desired_w = self.goal_angle_rad / self.horizon
                score_heading = -abs(w - desired_w)

                score_speed = v

                score = 3.0 * score_speed + 0.6 * score_heading - obstacle_cost * 1.0

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w
        self.debug_angle_marker.publish(self.display_arrow_from_angle(best_w, best_v))
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.get_logger().info(f"CMD: {cmd}")
        self.local_publisher.publish(cmd)

    def min_distance_along_trajectory(self, v, w):
        steps = int(self.horizon / self.dt)
        x, y, yaw = 0., 0., 0.

        angles = self.last_scan.angle_min + np.arange(len(self.last_scan.ranges)) * self.last_scan.angle_increment
        ranges = np.array(self.last_scan.ranges)
        ranges[ranges < 0.19] = float('inf')
        min_dist = float("inf")

        for idx in range(steps):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt

            px = ranges * np.cos(angles)
            py = ranges * np.sin(angles)

            d = np.sqrt((px - x)**2 + (py - y)**2)
            min_dist = min(min_dist, np.min(d))
        return min_dist
    
    def get_distance_cost(self, v, w):
        min_dist = self.min_distance_along_trajectory(v, w) - self.robot_radius
        if min_dist <= 0:
            return 1e6
        else:
            return 1. / min_dist

    def trajectory_safe(self, v, w):
        steps = int(self.horizon / self.dt)
        self.get_logger().info(f"VW: {v} | {w}")
        x, y, yaw = 0., 0., 0.

        angles = self.last_scan.angle_min + np.arange(len(self.last_scan.ranges)) * self.last_scan.angle_increment
        ranges = np.array(self.last_scan.ranges)
        ranges[ranges < 0.19] = float('inf')

        for idx in range(steps):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt

            px = ranges * np.cos(angles)
            py = ranges * np.sin(angles)

            d = np.sqrt((px - x)**2 + (py - y)**2)
            if idx == steps - 1:
                self.get_logger().info(f"xyy: {x} : {y} : {yaw}")
                self.get_logger().info(f"D: {d}")
            if np.any(d < self.robot_radius):
                return False
        return True

    def set_scan(self, msg):
        self.last_scan = msg
        arr = np.array(msg.ranges)
        n = len(arr)
        arr = np.roll(arr, n // 2)
        self.last_scan.ranges = arr
        self.last_scan.angle_min = -np.pi
        # valid_indicies = np.where(arr > 0.19)[0]
        # if len(valid_indicies) == 0:
        #     return
        # closest_idx = valid_indicies[np.argmin(arr[valid_indicies])]
        # closest_dist = arr[closest_idx]
        # closest_angle = self.last_scan.angle_min + closest_idx * msg.angle_increment
        # self.get_logger().info(f"Closest obstacle: {closest_dist:.2f} m at {closest_angle:.2f} rad")

    def smooth_angle(self, current_angle, alpha=0.2):
        px, py = math.cos(self.goal_angle_rad), math.sin(self.goal_angle_rad)
        cx, cy = math.cos(current_angle), math.sin(current_angle)

        x = (1-alpha)*px + alpha*cx
        y = (1-alpha)*py + alpha*cy

        return math.atan2(y, x)

    def set_target(self, msg):
        self.last_msg_time = time.time()
        # inverse goal direction angle.
        new_angle = -msg.angle_from_center_rad
        if self.goal_angle_rad is None:
            self.goal_angle_rad = new_angle
        else:
            self.goal_angle_rad = self.smooth_angle(new_angle)
        self.goal_depth_m = msg.depth_m

    def display_arrow_from_angle(self, angle, distance):
        half = angle / 2.0
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "points"
        marker.id = 150
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.z = math.sin(half)
        marker.pose.orientation.w = math.sin(half)
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.g = 1.0
        return marker


def main(args=None):
    try:
        with rclpy.init(args=args):
            dynamic_window_planner = DynamicWindowPlanner()

            rclpy.spin(dynamic_window_planner)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()