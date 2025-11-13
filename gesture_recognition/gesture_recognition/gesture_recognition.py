# keep venv
from . import venv_hack

# ros includes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.time import Time

# Other includes
import onnxruntime as ort
import numpy as np

# Message includes
from std_msgs.msg import String
from mediapipe_msg.msg import PoseStamped

IDX_TO_CLASS_MAPPING = {0: 'follow', 1: 'other', 2: 'stop'}


class GestureRecognizer(Node):

    def __init__(self):
        super().__init__('gesture_recognizer')
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/landmark/normalized_pose',
            self.pose_parser,
            10)
        self.pose_subscriber
        self.bridge = CvBridge()
        self.gesture_publisher = self.create_publisher(String, '/gesture/gesture_as_str', 10)
        self.ort_session = ort.InferenceSession("/home/GTL/jmagana/gte/ml/TurtlebotFollower/gesture_mlp.onnx")

    def landmark_to_float(self, landmark):
        return [landmark.x, landmark.y, landmark.z, landmark.visibility, landmark.presence]

    def pose_parser(self, msg):
        landmarks = []
        for landmark in msg.pose.landmarks:
            landmarks.extend(self.landmark_to_float(landmark))
        landmarks = np.array(landmarks, dtype=np.float32)
        print(landmarks.shape)
        input_landmarks = np.expand_dims(landmarks, 0)
        outputs = self.ort_session.run(None, {"input": input_landmarks})
        logits = outputs[0]
        pred_class = IDX_TO_CLASS_MAPPING[int(np.argmax(logits, axis=1)[0])]
        print(f"Predicted class: ", pred_class)

def main(args=None):
    try:
        with rclpy.init(args=args):
            gesture_recognizer = GestureRecognizer()

            rclpy.spin(gesture_recognizer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()