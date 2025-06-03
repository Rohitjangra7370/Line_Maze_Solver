#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            'camera_sensor/image_raw',
            self.image_callback,
            10)
        self.image_pub = self.create_publisher(Image, 'processed_image', 10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape

            # Convert to HSV and threshold for white
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Draw all contours in bright green with thick lines
                cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 4)
                largest = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    # Raise the red dot by 20% of the image height
                    cy = max(0, cy - int(0.2 * height))
                    # Draw a circle at the new centroid
                    cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
            else:
                self.get_logger().info("No contours found.")

            # Publish the processed image
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
