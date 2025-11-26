# keep venv
from . import venv_hack

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from tracking_msg.msg import TrackingImage
from cv_bridge import CvBridge, CvBridgeError

import cv2
from ultralytics import YOLO


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo')
        self.img_subscription = self.create_subscription(
            Image,
            '/astra/color/image_raw',
            self.process_img,
            10)
        self.img_subscription
            
        self.debug_annotated_image_publisher = self.create_publisher(Image, '/debug/annotated_image', 10)
        self.debug_cropped_image_publisher = self.create_publisher(Image, '/debug/croped_image', 10)
        self.tracked_image_publisher = self.create_publisher(TrackingImage, '/tracking/tracked_image', 10)

        self.bridge = CvBridge()

        self.model = YOLO("yolo11n.pt").to("cpu")
        self.classes = [0]
        self.tracker_config = "/home/GTL/jmagana/gte/ml/TurtlebotFollower/botsort.yaml"

    def process_img(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Error converting frame skipping")
            return
        results = self.model.track(
            frame,
            tracker=self.tracker_config,
            classes=self.classes,
            stream=False,
            verbose=False,
        )

        result = results[0]
        boxes = result.boxes
        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                track_id = int(box.id.item()) if box.id is not None else -1
                cropped_image = frame[int(y1):int(y2), int(x1):int(x2), :]
                cv2.rectangle(
                    frame,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2
                )
                cv2.putText(
                    frame,
                    f"ID {track_id}",
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
                original_height, original_width, _ = frame.shape
                tracked_image = TrackingImage()
                tracked_image.header.frame_id = msg.header.frame_id
                tracked_image.header.stamp = self.get_clock().now().to_msg()
                tracked_image.frame = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")
                self.debug_cropped_image_publisher.publish(tracked_image.frame)
                tracked_image.id = track_id
                tracked_image.x1 = int(x1)
                tracked_image.y1 = int(y1)
                tracked_image.x2 = int(x2)
                tracked_image.y2 = int(y2)
                tracked_image.original_width = original_width
                tracked_image.original_height = original_height
                self.tracked_image_publisher.publish(tracked_image)
        out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.debug_annotated_image_publisher.publish(out_msg)



def main(args=None):
    try:
        with rclpy.init(args=args):
            yolo_node = YoloNode()

            rclpy.spin(yolo_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()