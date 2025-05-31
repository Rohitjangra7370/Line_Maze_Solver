#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/line_follower/image_raw',
            self.image_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.Kp = 0.002
        self.last_error = 0
        self.get_logger().info("Line Follower Node Initialized")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape

            # Crop lower part of the image for line detection
            crop_height = 40
            crop = cv_image[height-crop_height:height, 0:width]

            # Convert to HSV and threshold for black
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 50))

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    error = cx - width // 2

                    # PID control (simple P)
                    twist = Twist()
                    twist.linear.x = 0.12
                    twist.angular.z = -float(error) * self.Kp
                    self.cmd_vel_pub.publish(twist)
                    self.last_error = error
                    return

            # If no line is found, stop
            self.cmd_vel_pub.publish(Twist())

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
