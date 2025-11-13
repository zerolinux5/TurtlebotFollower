# ros includes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# Other includes
from enum import Enum
from collections import deque

# Message includes
from follow_msg.msg import Target
from std_msgs.msg import String

IDX_TO_CLASS_MAPPING = {'follow': 0, 'other': 1, 'stop': 2}

class State(Enum):
    IDLE = 0
    FOLLOW = 1
    STOP = 2


class StateMachine(Node):

    def __init__(self):
        super().__init__('state_machine')
        self.gesture_subscriber = self.create_subscription(
            String,
            '/gesture/gesture_as_str',
            self.parse_gesture,
            10)
        self.target_subscription = self.create_subscription(
            Target,
            '/follow/target',
            self.process_target,
            10)
        self.gesture_subscriber
        self.target_subscription
        self.command_target_publisher = self.create_publisher(Target, '/command/target', 10)
        self.current_state = State.IDLE
        self.past_gestures = deque([])
        self.limit = 5

    def process_target(self, msg):
        if self.current_state == State.FOLLOW:
            self.command_target_publisher.publish(msg)

    def parse_gesture(self, msg):
        self.past_gestures.append(msg.data)
        if len(self.past_gestures) > self.limit:
            self.past_gestures.popleft()
        if len(self.past_gestures) == self.limit:
            first_gesture = self.past_gestures[0]
            is_valid_reading = True
            for gesture in self.past_gestures:
                if gesture != first_gesture:
                    is_valid_reading = False
                    break
            if is_valid_reading:
                match (first_gesture):
                    case "follow":
                        if self.current_state != State.FOLLOW:
                            print("Changing to Follow")
                            self.current_state = State.FOLLOW
                    case "stop":
                        if self.current_state != State.STOP:
                            print("Changing to Stop")
                            zeroed_target = Target()
                            zeroed_target.header.stamp = self.get_clock().now().to_msg()
                            zeroed_target.header.frame_id = "base_link"
                            zeroed_target.angle_from_center_rad = 0.
                            zeroed_target.depth_m = 0.
                            self.command_target_publisher.publish(zeroed_target)
                            self.current_state = State.STOP
                    case _:
                        pass

def main(args=None):
    try:
        with rclpy.init(args=args):
            state_machine = StateMachine()

            rclpy.spin(state_machine)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()