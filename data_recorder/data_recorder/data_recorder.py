import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import sys, select, termios, tty, os
import threading

from mediapipe_msg.msg import PoseStamped
from sensor_msgs.msg import Image

class DataRecorder(Node):

    def __init__(self):
        super().__init__('data_recorder')
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/landmark/normalized_pose',
            self.save_pose,
            10)
        self.debug_image_subscription = self.create_subscription(
            Image,
            '/landmark/debug_image',
            self.save_debug,
            10)
            
        self.target_publisher = self.create_publisher(PoseStamped, '/debug/recorded_frame', 10)
        self.pose_subscription
        self.debug_image_subscription
        self.pose = None
        self.debug_image = None
        self.running = True
        self.thread = threading.Thread(target=self.listen_keys)
        self.thread.start()
        self.root = "data"
        self.file = 'data.csv'


    def listen_keys(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(fd)
            while self.running:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    self.handle_key(key)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def handle_key(self, key):
        if not self.pose:
            print("Missing Pose")
        if not self.debug_image:
            print("Missing Debug Image")
        if not self.pose or not self.debug_image:
            return
        is_synced, delta = self.is_synced(self.pose, self.debug_image)
        if not is_synced:
            print("Msgs not synced by: ", delta)
            return
        dir_name = None
        match key:
            case "f":
                dir_name = "follow"
                print("follow")
            case "s":
                dir_name = "stop"
                print("stop")
            case "m":
                dir_name = "me"
                print("me")
            case "o":
                dir_name = "other"
                print("other")
            case _:
                self.running = False
                return
        full_path = os.path.join(self.root, dir_name, self.file)
        data = ", ".join([self.landmark_to_str(landmark) for landmark in self.pose.pose.landmarks]) + "\n"
        data_len = sum([self.landmark_to_len() for _ in self.pose.pose.landmarks])
        print(f"Writing to: {full_path}")
        print(f"Data of len {data_len}")
        with open(full_path, "a") as fd:
            fd.write(data)

    def save_pose(self, msg):
        self.pose = msg

    def save_debug(self, msg):
        self.debug_image = msg
    
    def is_synced(self, msg1, msg2, tolerance=0.05):
        t1 = msg1.header.stamp.sec + msg1.header.stamp.nanosec * 1e-9
        t2 = msg2.header.stamp.sec + msg2.header.stamp.nanosec * 1e-9
        return abs(t1 - t2) < tolerance, abs(t1 - t2)

    def landmark_to_len(self):
        return 5
    
    def landmark_to_str(self, landmark):
        return f"{landmark.x}, {landmark.y}, {landmark.z}, {landmark.visibility}, {landmark.presence}"


def main(args=None):
    try:
        with rclpy.init(args=args):
            data_recorder = DataRecorder()
            rclpy.spin(data_recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    data_recorder.running = False
    data_recorder.thread.join()
    data_recorder.destroy_node()


if __name__ == '__main__':
    main()
