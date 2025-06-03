#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import time

class OptimizedLineFollower(Node):
    def __init__(self):
        super().__init__('optimized_line_follower')
        
        # Initialize cv_bridge once
        self.bridge = CvBridge()
        
        # Performance monitoring
        self.frame_count = 0
        self.last_time = time.time()
        self.fps_buffer = deque(maxlen=30)
        
        # Optimized QoS profile for image processing
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Keep only latest image
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriptions and publishers
        self.image_sub = self.create_subscription(
            Image,
            'processed_image',
            self.image_callback,
            image_qos)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            QoSProfile(depth=1))
        
        # Control parameters - tuned for stability
        self.Kp = 0.08
        self.Ki = 0.003
        self.Kd = 0.05
        self.last_error = 0
        self.integral_error = 0
        self.error_history = deque(maxlen=5)
        
        # Image processing parameters
        self.crop_height = 60
        self.processing_width = 320  # Reduced resolution for faster processing
        self.processing_height = 240
        
        # Pre-allocated arrays for performance
        self.mask = None
        self.crop = None
        
        # Line detection parameters
        self.min_contour_area = 100
        self.max_linear_speed = 1
        self.max_angular_speed = 1.0
        
        # Moving average filter for smooth control
        self.error_filter = deque(maxlen=3)
        
        # Performance monitoring timer
        self.create_timer(2.0, self.log_performance)
        
        self.get_logger().info("Optimized Line Follower Node Initialized")

    def preprocess_image(self, cv_image):
        """Optimized image preprocessing with minimal memory allocation"""
        height, width = cv_image.shape[:2]
        
        # Resize image for faster processing if needed
        if width > self.processing_width:
            scale_factor = self.processing_width / width
            new_height = int(height * scale_factor)
            cv_image = cv2.resize(cv_image, (self.processing_width, new_height), 
                                interpolation=cv2.INTER_LINEAR)
            height, width = new_height, self.processing_width
        
        # Crop region of interest (lower portion for line detection)
        crop_start = max(0, height - self.crop_height)
        crop = cv_image[crop_start:height, 0:width]
        
        return crop, width

    def detect_line_optimized(self, crop, width):
        """Optimized line detection using efficient computer vision techniques"""
        # Convert to grayscale for faster processing
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        
        # Use adaptive thresholding for better performance under varying conditions
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY_INV, 11, 2)
        
        # Morphological operations to clean up the binary image
        kernel = np.ones((3,3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Find contours with optimized retrieval mode
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, binary
        
        # Filter contours by area and select the largest valid one
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_contour_area]
        
        if not valid_contours:
            return None, binary
        
        # Find the largest contour (assumed to be the line)
        largest_contour = max(valid_contours, key=cv2.contourArea)
        
        # Calculate centroid using moments
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            return cx, binary
        
        return None, binary

    def calculate_control_command(self, error, width):
        """Advanced PID controller with derivative and integral terms"""
        if error is None:
            return Twist()  # Stop if no line detected
        
        # Normalize error to [-1, 1] range
        normalized_error = error / (width / 2.0)
        
        # Add to error history for filtering
        self.error_filter.append(normalized_error)
        filtered_error = sum(self.error_filter) / len(self.error_filter)
        
        # PID calculations
        self.integral_error += filtered_error
        self.integral_error = np.clip(self.integral_error, -10, 10)  # Prevent windup
        
        derivative_error = filtered_error - self.last_error
        
        # PID output
        pid_output = (self.Kp * filtered_error + 
                     self.Ki * self.integral_error + 
                     self.Kd * derivative_error)
        
        # Create twist message with speed adaptation
        twist = Twist()
        
        # Adaptive speed based on error magnitude
        error_magnitude = abs(filtered_error)
        speed_factor = max(0.3, 1.0 - error_magnitude * 0.7)
        
        twist.linear.x = self.max_linear_speed * speed_factor
        twist.angular.z = np.clip(-pid_output, -self.max_angular_speed, self.max_angular_speed)
        
        self.last_error = filtered_error
        return twist

    def image_callback(self, msg):
        """Optimized image processing callback with error handling and performance monitoring"""
        try:
            start_time = time.time()
            
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Preprocess image
            crop, width = self.preprocess_image(cv_image)
            
            # Detect line
            cx, binary = self.detect_line_optimized(crop, width)
            
            # Calculate error and control command
            error = (cx - width // 2) if cx is not None else None
            twist = self.calculate_control_command(error, width)
            
            # Publish control command
            self.cmd_vel_pub.publish(twist)
            
            # Performance monitoring
            processing_time = time.time() - start_time
            self.fps_buffer.append(1.0 / processing_time if processing_time > 0 else 0)
            self.frame_count += 1
            
            # Log detailed information periodically
            if self.frame_count % 50 == 0:
                avg_fps = sum(self.fps_buffer) / len(self.fps_buffer) if self.fps_buffer else 0
                self.get_logger().info(
                    f"Processing: {processing_time*1000:.1f}ms, "
                    f"FPS: {avg_fps:.1f}, "
                    f"Error: {error if error else 'N/A'}, "
                    f"Speed: {twist.linear.x:.2f}, "
                    f"Turn: {twist.angular.z:.2f}"
                )

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
            # Publish stop command on error
            self.cmd_vel_pub.publish(Twist())

    def log_performance(self):
        """Performance monitoring callback"""
        current_time = time.time()
        time_diff = current_time - self.last_time
        
        if time_diff > 0:
            fps = self.frame_count / time_diff
            avg_processing_fps = sum(self.fps_buffer) / len(self.fps_buffer) if self.fps_buffer else 0
            
            self.get_logger().info(
                f"Node Performance - Callback FPS: {fps:.1f}, "
                f"Processing FPS: {avg_processing_fps:.1f}, "
                f"Frames processed: {self.frame_count}"
            )
        
        self.frame_count = 0
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    # Enable optimizations
    try:
        # Create node with optimized settings
        node = OptimizedLineFollower()
        
        # Use optimized spinning with reduced overhead
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start optimized line follower: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
