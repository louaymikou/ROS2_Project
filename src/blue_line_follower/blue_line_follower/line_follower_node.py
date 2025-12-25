#!/usr/bin/env python3
"""
Blue Line Follower Node
Based on the algorithm from the Jupyter notebook training material.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# Linear forward speed
LINEAR_SPEED = 0.2

# Proportional constant to be applied on speed when turning 
# (Multiplied by the error value)
KP = 1.5/100


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Line Follower Node has been started')

    def get_contour_data(self, mask):
        """
        Return the centroid of the largest contour in the binary image 'mask' (the line) 
        """
        # Constants
        MIN_AREA_TRACK = 50  # Minimum area for track marks

        # Get a list of contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        line = {}

        for contour in contours:
            M = cv2.moments(contour)

            if M['m00'] > MIN_AREA_TRACK:
                # Contour is part of the track
                line['x'] = int(M["m10"]/M["m00"])
                line['y'] = int(M["m01"]/M["m00"])

        return line

    def image_callback(self, data):
        """
        Callback function for processing camera images
        """
        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            # Convert BGR to HSV
            hsv_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

            # Define range of blue color in HSV
            lower_blue = np.array([100, 50, 50])   # Lower bound of blue color
            upper_blue = np.array([130, 255, 255])  # Upper bound of blue color

            # Create a binary mask
            blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

            # Apply the mask to the original image
            blue_segmented_image = cv2.bitwise_and(current_frame, current_frame, mask=blue_mask)

            # Detect line and get its centroid
            line = self.get_contour_data(blue_mask)

            # Move depending on detection 
            cmd = Twist()
            height, width, _ = blue_segmented_image.shape
            
            error = 0
            
            if line:
                x = line['x']
                error = x - width//2
                cmd.linear.x = LINEAR_SPEED
                
                # Draw circle on detected line for visualization (optional)
                cv2.circle(blue_segmented_image, (line['x'], line['y']), 5, (0, 0, 255), 7)
            else:
                # No line detected, stop
                cmd.linear.x = 0.0
                self.get_logger().warn('No blue line detected!', throttle_duration_sec=1.0)
            
            # Determine the speed to turn and get the line in the center of the camera
            cmd.angular.z = float(error) * -KP
            
            # Log the error and angular velocity (throttled to avoid spam)
            if line:
                self.get_logger().debug(f"Error: {error} | Angular Z: {cmd.angular.z}")

            # Send the command to execute
            self.publisher.publish(cmd)
            
            # Display the processed image with line detection
            cv2.imshow("Blue Segmented Image", blue_segmented_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollowerNode()
    
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        line_follower.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
