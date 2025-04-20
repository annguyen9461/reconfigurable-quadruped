#!/usr/bin/env python3

import os
import rclpy
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory  # Find package path dynamically


class YoloNode(Node):
    """
    Use Yolo to identify scene objects

    Subscribes
    ----------
    image (sensor_msgs/msg/Image) - The input image

    Publishes
    ---------
    new_image (sensor_msgs/msg/Image) - The image with the detections

    Parameters
    model (string) - The Yolo model to use: see docs.ultralytics.org for available values. Default is yolo11n.pt
    """
    def __init__(self):
        super().__init__("pose")
        self.bridge = CvBridge()
        
        # Get package path dynamically
        package_path = get_package_share_directory('quad_vision')

        # Path to trained model (best.pt)
        model_path = os.path.join(package_path, "models", "best.pt")

        # Log model path
        self.get_logger().info(f"Using YOLO model: {model_path}")

        # Load trained model
        self.model = YOLO(model_path)

        self.create_subscription(Image, 'image', self.yolo_callback, 10)
        self.pub_yolo_img = self.create_publisher(Image, 'yolo_image', 10)
        self.pub_num_pins = self.create_publisher(Int32, 'num_bowling_pins', 10)

    def yolo_callback(self, image):
        """Identify all the objects in the scene"""
        # Convert ROS2 Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        
        # Run the YOLO model
        results = self.model(cv_image)
        detections = results[0]  # Get first batch of detections

        # Extract detected bounding boxes
        boxes = detections.boxes  # YOLOv8 bounding box results
        class_names = detections.names  # Dictionary {0: 'bowling-ball', 1: 'bowling-pins', 2: 'sweep board'}

        # Ensure there are detections
        if boxes is not None:
            # Convert to a NumPy array for easy filtering
            class_ids = boxes.cls.cpu().numpy().astype(int)  # Get detected class IDs
            confidences = boxes.conf.cpu().numpy()  # Get confidence scores

            # Count bowling pins with confidence > 0.7
            bowling_pins_count = sum((class_ids == 1) & (confidences > 0.7))

            # Publish the count as an Int32 message
            count_msg = Int32()
            count_msg.data = int(bowling_pins_count)
            self.pub_num_pins.publish(count_msg)

        # Draw detections on the image
        frame = detections.plot()

        # Convert back to ROS2 image message
        new_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # Publish the annotated image
        self.pub_yolo_img.publish(new_msg)


def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()