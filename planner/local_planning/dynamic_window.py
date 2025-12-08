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
        # subscribers and publishers
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

        # parameters
        self.declare_parameter('robot_radius', 0.21)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('horizon', 3.0)
        self.declare_parameter('speed_weight', 1.0)
        self.declare_parameter('score_weight', 1.5)
        self.declare_parameter('obstacle_weight', 0.1)
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('v_max', 0.11)
        self.declare_parameter('v_step', 20)
        self.declare_parameter('w_min', -0.45)
        self.declare_parameter('w_max', 0.45)
        self.declare_parameter('w_step', 30)

        # Local variables
        self.last_scan = None
        self.last_msg_time = time.time()
        self.timer = self.create_timer(0.1, self.update)
        self.goal_angle_rad = 0.0
        self.goal_depth_m = None

        # variables based off parameters
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.horizon = self.get_parameter('horizon').get_parameter_value().double_value
        self.speed_weight = self.get_parameter('speed_weight').get_parameter_value().double_value
        self.score_weight = self.get_parameter('score_weight').get_parameter_value().double_value
        self.obstacle_weight = self.get_parameter('obstacle_weight').get_parameter_value().double_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.v_step = self.get_parameter('v_step').get_parameter_value().integer_value
        self.w_min = self.get_parameter('w_min').get_parameter_value().double_value
        self.w_max = self.get_parameter('w_max').get_parameter_value().double_value
        self.w_step = self.get_parameter('w_step').get_parameter_value().integer_value
        
        self.v_samples = np.linspace(self.v_min, self.v_max, self.v_step)
        self.w_samples = np.linspace(self.w_min, self.w_max, self.w_step)

    def update(self):
        # self.get_logger().info(f"Inside Heading: {np.degrees(self.goal_angle_rad):.2f} deg {self.goal_angle_rad} rad target Depth: {self.goal_depth_m}")
        # No lidar scan topic
        if self.last_scan is None:
            return
        
        # At target stop or target doesn't make sense
        if self.goal_depth_m is None or self.goal_depth_m < 1.1:
            self.get_logger().info(f"Publishing Zero")
            self.local_publisher.publish(Twist())
            return

        best_score = -1e9
        best_v, best_w = 0.0, 0.0
        # desired_w = self.goal_angle_rad / self.horizon
        desired_w = self.goal_angle_rad
        # self.get_logger().info(f"Target Heading: {np.degrees(self.goal_angle_rad):.2f} deg {self.goal_angle_rad} rad target: {desired_w}")
        steps = int(self.horizon / self.dt)

        delta_steps = np.arange(steps) * self.dt
        for v in self.v_samples:
            for w in self.w_samples:
                obstacle_cost = self.get_distance_cost(v, w, delta_steps)
                
                score_heading = -abs(w - desired_w)

                score_speed = v
                weighted_speed = self.speed_weight * score_speed
                weighted_score = self.score_weight * score_heading
                weighted_obstacle_cost = self.obstacle_weight * obstacle_cost
                score = weighted_speed + weighted_score - weighted_obstacle_cost
                # self.get_logger().info(f"VW: {v:.4f} | {w:.4f}  | Total: {score:.3f} Speed: {weighted_speed:.3f} Score: {weighted_score:.3f} Obstacle: {weighted_obstacle_cost:.3f}")

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w
        # self.get_logger().info(f"BestVW: {best_v:.4f} | {best_w:.4f}")
        self.debug_angle_marker.publish(self.display_arrow_from_angle(best_w, best_v))
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        # self.get_logger().info(f"CMD: {cmd}")
        self.local_publisher.publish(cmd)

    def min_distance_along_trajectory(self, v, w, delta_steps):
        yaw = w * delta_steps
        x_traj = v * np.cos(yaw) * delta_steps
        y_traj = v * np.sin(yaw) * delta_steps
        dx = self.laser_x[None, :] - x_traj[:, None]
        dy = self.laser_y[None, :] - y_traj[:, None]
        dist = np.sqrt(dx**2 + dy**2)
        min_dist = np.min(dist)
        return min_dist
    
    def get_distance_cost(self, v, w, delta_steps):
        min_dist = self.min_distance_along_trajectory(v, w, delta_steps) - self.robot_radius
        if min_dist <= 0:
            return 1e6
        else:
            return 1. / min_dist

    def set_scan(self, msg):
        self.last_scan = msg
        arr = np.array(msg.ranges)
        n = len(arr)
        arr = np.roll(arr, n // 2)
        self.last_scan.ranges = arr
        self.last_scan.angle_min = -np.pi

        self.laser_ranges = np.array(self.last_scan.ranges)
        self.laser_ranges[self.laser_ranges < 0.19] = float('inf')
        self.laser_angles = self.last_scan.angle_min + np.arange(len(self.last_scan.ranges)) * self.last_scan.angle_increment
        self.laser_x = self.laser_ranges * np.cos(self.laser_angles)
        self.laser_y = self.laser_ranges * np.sin(self.laser_angles)

    def smooth_angle(self, current_angle, alpha=0.2):
        px, py = math.cos(self.goal_angle_rad), math.sin(self.goal_angle_rad)
        cx, cy = math.cos(current_angle), math.sin(current_angle)

        x = (1-alpha)*px + alpha*cx
        y = (1-alpha)*py + alpha*cy

        return math.atan2(y, x)

    def set_target(self, msg):
        # self.get_logger().info(f"Received Heading: {np.degrees(self.goal_angle_rad):.2f} deg {self.goal_angle_rad} rad target Depth: {self.goal_depth_m}")
        self.last_msg_time = time.time()
        # inverse goal direction angle.
        new_angle = -msg.angle_from_center_rad
        if self.goal_angle_rad is None:
            self.goal_angle_rad = new_angle
        else:
            self.goal_angle_rad = self.smooth_angle(new_angle, 0.7)
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
        marker.scale.x = 3.0 * distance
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
