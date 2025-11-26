import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from visualization_msgs.msg import Marker, MarkerArray
from follow_msg.msg import Target
from mediapipe_msg.msg import PoseStamped

import numpy as np
import time
import uuid

H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]
], dtype=float)
R = np.eye(2) * (0.05**2)
Q_base = np.array([
        [1/3, 0, 1/2,  0],
        [0, 1/3,  0, 1/2],
        [1/2, 0,  1,  0],
        [0, 1/2,  0,  1]
    ], dtype=float) * 5
chi_distribution = 5.991


def make_A(dt):
    return np.array([
        [1, 0, dt,  0],
        [0, 1,  0, dt],
        [0, 0,  1,  0],
        [0, 0,  0,  1]
    ], dtype=float)

def get_s(P):
    S = H @ P @ H.T + R
    return np.linalg.inv(S)

def predict(x, P, dt):
    A = make_A(dt)
    dt_3 = dt**3
    dt_2 = dt**2
    T = np.array([
        [dt_3, dt_3, dt_2,  dt_2],
        [dt_3, dt_3, dt_2,  dt_2],
        [dt_2, dt_2, dt,  dt],
        [dt_2, dt_2, dt,  dt]
    ], dtype=float)
    Q = Q_base * T
    x = A @ x
    P = A @ P @ A.T + Q
    return x, P

def update(x, P, z):
    K = P @ H.T @ get_s(P)
    x = x + (K @ (z - (H @ x)))
    P = (np.eye(4) - K @ H) @ P
    return x, P

class Person:
    def __init__(self, x, y, id_):
        self.x = np.array([[x], [y], [0], [0]], dtype=np.float32)
        self.P = Q_base
        self.id = id_
        self.last_marked = time.time()
        self.times_read = 0

    def get_d_2(self, z):
        y = z - (H @ self.x)
        S = get_s(self.P)
        d_2 = y.T @ S @ y
        return d_2
    
    def update(self, x, P, measured=False):
        self.x = x
        self.P = P
        if measured:
            self.last_marked = time.time()
            self.times_read += 1

    def is_valid(self):
        tolerance = 5
        return self.times_read >= tolerance

    def should_delete(self, curr_time):
        tolerance = 3
        return curr_time - self.last_marked >= tolerance


class KalmanFilter(Node):

    def __init__(self):
        super().__init__('kalman_filter')
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/landmark/world_pose',
            self.pose_processor,
            10)
        self.pose_subscription

        self.tracking = {}
        self.prev_time = time.time()
        self.timer = self.create_timer(0.1, self.update_loop)
            
        # self.world_pose_publisher = self.create_publisher(PoseStamped, '/landmark/world_pose', 10)

    def update_loop(self):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        mark_for_del = []
        for id_, person in self.tracking.items():
            if person.should_delete(now):
                mark_for_del.append(id_)
                continue
            x, P = predict(person.x, person.P, dt)
            person.update(x, P)
        for id_ in mark_for_del:
            del self.tracking[id_]

    def pose_processor(self, msg):
        xs = []
        ys = []
        is_valid = False
        for landmark in msg.pose.landmarks:
            if landmark.is_world_valid:
                is_valid = True
                xs.append(landmark.world_x)
                ys.append(landmark.world_y)
        if not is_valid:
            return
        final_x = np.median(np.array(xs))
        final_y = np.median(np.array(ys))
        z = np.array([
            [final_x],
            [final_y]
        ], dtype=float)
        person_id = None
        smallest_d_2 = float("inf")
        for id_, person in self.tracking.items():
            d_2 = person.get_d_2(z)
            if d_2 < smallest_d_2:
                person_id = id_
                smallest_d_2 = d_2
        if smallest_d_2 <= chi_distribution:
            person = self.tracking[person_id]
            x, P = update(person.x, person.P, z)
            person.update(x, P, True)
        else:
            person_id = str(uuid.uuid4())
            person = Person(final_x, final_y, person_id)
            self.tracking[person_id] = person
        print("Num: ", len(self.tracking))
        for person in self.tracking.values():
            if person.is_valid():
                print(f"X: {person.x[0]}, Y: {person.x[1]}")


def main(args=None):
    try:
        with rclpy.init(args=args):
            kalman_filter = KalmanFilter()

            rclpy.spin(kalman_filter)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()