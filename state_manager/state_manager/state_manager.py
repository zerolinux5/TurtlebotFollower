# ros includes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# Other includes
from enum import Enum
from collections import deque
import time

# Message includes
from follow_msg.msg import Target
from mediapipe_msg.msg import Gesture

IDX_TO_CLASS_MAPPING = {'follow': 0, 'other': 1, 'stop': 2}

class State(Enum):
    IDLE = 0
    FOLLOW = 1
    STOP = 2

class TrackedPerson:
    def __init__(self, id_):
        self.id = id_
        self.data = deque([])
        self.last_seen = time.time()

    def append_data(self, data):
        self.last_seen = time.time()
        if self.is_full():
            self.remove_first()
        self.data.append((data, self.last_seen))

    def get_size(self):
        return len(self.data)

    def is_empty(self):
        return len(self.data) == 0

    def is_full(self):
        full_size = 5
        return len(self.data) == full_size
    
    def remove_first(self):
        self.data.popleft()

    def get_first_data(self):
        return self.get_first()[0]

    def get_first(self):
        return self.data[0]

    def is_old(self):
        now = time.time()
        tolerance = 3
        return (now - self.last_seen) > tolerance
    
    def is_all_same(self):
        first = self.get_first_data()
        for other in self.data:
            other_data, _ = other
            if first != other_data:
                return False
        return True
    
    def clear_old_data(self):
        now = time.time()
        tolerance = 5.0
        while True:
            if self.is_empty():
                break
            _, gesture_time = self.get_first()
            if (now - gesture_time) > tolerance:
                self.remove_first()
            else:
                break


class StateMachine(Node):

    def __init__(self):
        super().__init__('state_machine')
        self.gesture_subscriber = self.create_subscription(
            Gesture,
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
        self.past_gestures = {}
        self.current_person_id = None
        # At rate of 1hz loop
        self.timer = self.create_timer(1.0, self.update_loop)

    def update_loop(self):
        mark_for_del = []
        for person_id in self.past_gestures:
            person = self.past_gestures[person_id]
            if person.is_old():
                mark_for_del.append(person_id)
            else:
                person.clear_old_data()
        for person_id in mark_for_del:
            del self.past_gestures[person_id]

    def process_target(self, msg):
        if self.current_state == State.FOLLOW and self.current_person_id and msg.id == self.current_person_id:
            self.command_target_publisher.publish(msg)

    def parse_gesture(self, msg):
        person_id = msg.id
        # ignore other gestures if we have a target
        if self.current_person_id and self.current_person_id != person_id:
            return
        if person_id not in self.past_gestures:
            self.past_gestures[person_id] = TrackedPerson(person_id)
        tracked_person = self.past_gestures[person_id]
        tracked_person.append_data(msg.gesture)
        if tracked_person.is_full():
            if tracked_person.is_all_same():
                first_gesture = tracked_person.get_first_data()
                match (first_gesture):
                    case "follow":
                        if self.current_state != State.FOLLOW:
                            self.get_logger().info(f"Changing to Follow: {person_id}")
                            self.current_state = State.FOLLOW
                            self.current_person_id = person_id
                    case "stop":
                        if self.current_state != State.STOP:
                            self.get_logger().info("Changing to Stop")
                            self.current_person_id = None
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