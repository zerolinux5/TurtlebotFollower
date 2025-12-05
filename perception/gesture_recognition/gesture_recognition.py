# keep venv
from . import venv_hack

# ros includes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Other includes
import onnxruntime as ort
import numpy as np
import os

# Message includes
from mediapipe_msg.msg import PoseStamped, Gesture

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
        self.gesture_publisher = self.create_publisher(Gesture, '/gesture/gesture_as_str', 10)
        model_path = os.path.join(
            get_package_share_directory('perception'),
            'models',
            'gesture_mlp.onnx'
        )
        self.ort_session = ort.InferenceSession(model_path)

    def landmark_to_float(self, landmark):
        return [landmark.normalized_x, landmark.normalized_y, landmark.normalized_z, landmark.visibility, landmark.presence, float(landmark.is_pixel_valid)]

    def pose_parser(self, msg):
        landmarks = []
        for landmark in msg.pose.landmarks:
            landmarks.extend(self.landmark_to_float(landmark))
        landmarks = np.array(landmarks, dtype=np.float32)
        input_landmarks = np.expand_dims(landmarks, 0)
        outputs = self.ort_session.run(None, {"input": input_landmarks})
        logits = outputs[0]
        pred_class = IDX_TO_CLASS_MAPPING[int(np.argmax(logits, axis=1)[0])]
        msg_as_str = Gesture()
        msg_as_str.id = msg.pose.id
        msg_as_str.gesture = pred_class
        self.gesture_publisher.publish(msg_as_str)
        self.get_logger().info(f"Predicted class: {pred_class}")

def main(args=None):
    try:
        with rclpy.init(args=args):
            gesture_recognizer = GestureRecognizer()

            rclpy.spin(gesture_recognizer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()