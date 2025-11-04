# keep venv
from . import venv_hack

# ros includes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

# Image includes
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
import cv2

# Other includes
from enum import Enum

# Message includes
from sensor_msgs.msg import Image
from mediapipe_msg.msg import Landmark, Pose

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
model_path = "pose_landmarker.task"
options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.IMAGE)

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

LandMarkEnum = Enum('LandMark', [
    ('nose', 0),
    ('left eye (inner)', 1),
    ('left eye', 2),
    ('left eye (outer)', 3),
    ('right eye (inner)', 4),
    ('right eye', 5),
    ('right eye (outer)', 6),
    ('left ear', 7),
    ('right ear', 8),
    ('mouth (left)', 9),
    ('mouth (right)', 10),
    ('left shoulder', 11),
    ('right shoulder', 12),
    ('left elbow', 13),
    ('right elbow', 14),
    ('left wrist', 15),
    ('right wrist', 16),
    ('left pinky', 17),
    ('right pinky', 18),
    ('left index', 19),
    ('right index', 20),
    ('left thumb', 21),
    ('right thumb',22),
    ('left hip', 23),
    ('right hip', 24),
    ('left knee', 25),
    ('right knee', 26),
    ('left ankle', 27),
    ('right ankle', 28),
    ('left heel', 29),
    ('right heel', 30),
    ('left foot index', 31),
    ('right foot index', 32),
])

class MediaPipe(Node):

    def __init__(self):
        super().__init__('mediapipe')
        self.img_subscriber = self.create_subscription(
            Image,
            '/astra/color/image_raw',
            self.img_parser,
            10)
        self.img_subscriber
        self.bridge = CvBridge()
        self.debug_publisher = self.create_publisher(Image, '/landmark/debug_image', 10)
        self.landmark_publisher = self.create_publisher(Pose, '/landmark/normalized_pose', 10)

    def img_parser(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            self.get_logger().info("Failed: ", str(e))
            return
        # cv_image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        with PoseLandmarker.create_from_options(options) as landmarker:
            results = landmarker.detect(mp_image)
            pose_landmarks_list = results.pose_landmarks
            for idx in range(len(pose_landmarks_list)):
                pose_landmarks = pose_landmarks_list[idx]
                pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                normalized_data = [landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks]

                pose_landmarks_proto.landmark.extend(normalized_data)
                mp_drawing.draw_landmarks(
                    image, pose_landmarks_proto, mp_pose.POSE_CONNECTIONS)

            debug_image = self.bridge.cv2_to_imgmsg(image, "rgb8")
            self.debug_publisher.publish(debug_image)
        for human in results.pose_landmarks:
            new_pose = Pose()
            for idx, landmark in enumerate(human):
                new_landmark = Landmark()
                new_landmark.name = LandMarkEnum(idx).name
                new_landmark.x = landmark.x
                new_landmark.y = landmark.y
                new_landmark.z = landmark.z
                new_landmark.visibility = landmark.visibility
                new_landmark.presence = landmark.presence
                new_pose.landmarks[idx] = new_landmark
            self.landmark_publisher.publish(new_pose)


def main(args=None):
    try:
        with rclpy.init(args=args):
            mediapipe = MediaPipe()

            rclpy.spin(mediapipe)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()