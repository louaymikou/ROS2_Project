#!/usr/bin/env python3
"""
Blue Line Follower Node with ArUco Detection and Bidirectional Control
Based on the algorithm from the Jupyter notebook training material.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# Linear forward speed
LINEAR_SPEED = 0.2

# PID constants for position error
KP = 0.008  # Proportional gain for position
KI = 0.0001  # Integral gain
KD = 0.005  # Derivative gain

# Orientation correction gain
KP_ANGLE = 0.002  # Proportional gain for angle correction

# Maximum angular velocity (rad/s)
MAX_ANGULAR_VEL = 1.5


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Track last detected ArUco to avoid spam
        self.last_detected_aruco = None
        self.last_aruco_time = 0
        
        # Direction control: True = forward (front camera), False = backward (rear camera)
        self.forward_direction = True
        
        # Movement control: robot only moves when enabled
        self.movement_enabled = False
        
        # Obstacle detection variables
        self.front_obstacle_distance = float('inf')  # Distance to front obstacle (m)
        self.rear_obstacle_distance = float('inf')   # Distance to rear obstacle (m)
        self.obstacle_threshold = 0.3  # Stop if obstacle closer than 30cm
        self.obstacle_detected = False
        
        # Subscribe to front camera topic
        self.front_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.front_camera_callback,
            10)
        
        # Subscribe to rear camera topic
        self.rear_subscription = self.create_subscription(
            Image,
            '/rear_camera/image_raw',
            self.rear_camera_callback,
            10)
        
        # Subscribe to front ultrasonic sensor
        self.front_ultrasonic_subscription = self.create_subscription(
            Range,
            '/front_ultrasonic/range',
            self.front_ultrasonic_callback,
            10)
        
        # Subscribe to rear ultrasonic sensor
        self.rear_ultrasonic_subscription = self.create_subscription(
            Range,
            '/rear_ultrasonic/range',
            self.rear_ultrasonic_callback,
            10)
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service to change direction
        self.direction_service = self.create_service(
            SetBool,
            'set_forward_direction',
            self.change_direction_callback)
        
        # Service to enable/disable movement
        self.movement_service = self.create_service(
            SetBool,
            'enable_movement',
            self.enable_movement_callback)
        
        # PID control variables
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        self.get_logger().info('Line Follower Node with Bidirectional Control has been started')
        self.get_logger().info('MOVEMENT IS DISABLED - Use: ros2 service call /enable_movement std_srvs/srv/SetBool "{data: true}" to start')
        self.get_logger().info('Forward: ros2 service call /set_forward_direction std_srvs/srv/SetBool "{data: true}"')
        self.get_logger().info('Backward: ros2 service call /set_forward_direction std_srvs/srv/SetBool "{data: false}"')
        self.get_logger().info(f'Obstacle detection enabled: Stop if closer than {self.obstacle_threshold}m')
    
    def front_ultrasonic_callback(self, msg):
        """
        Callback for front ultrasonic sensor
        """
        self.front_obstacle_distance = msg.range
        self.check_obstacle_status()
    
    def rear_ultrasonic_callback(self, msg):
        """
        Callback for rear ultrasonic sensor
        """
        self.rear_obstacle_distance = msg.range
        self.check_obstacle_status()
    
    def check_obstacle_status(self):
        """
        Check if there's an obstacle in the current direction of travel
        """
        was_blocked = self.obstacle_detected
        
        if self.forward_direction:
            # Going forward, check front sensor
            self.obstacle_detected = self.front_obstacle_distance < self.obstacle_threshold
        else:
            # Going backward, check rear sensor
            self.obstacle_detected = self.rear_obstacle_distance < self.obstacle_threshold
        
        # Log when obstacle status changes
        if self.obstacle_detected and not was_blocked:
            direction = "FRONT" if self.forward_direction else "REAR"
            distance = self.front_obstacle_distance if self.forward_direction else self.rear_obstacle_distance
            self.get_logger().warn(f'🚨 OBSTACLE DETECTED {direction}: {distance:.2f}m - Robot STOPPED!')
        elif not self.obstacle_detected and was_blocked:
            self.get_logger().info('✅ Obstacle cleared - Robot can continue')
    
    def get_obstacle_info(self):
        """
        Get current obstacle information for the active direction
        """
        if self.forward_direction:
            return self.obstacle_detected, self.front_obstacle_distance
        else:
            return self.obstacle_detected, self.rear_obstacle_distance
    
    def enable_movement_callback(self, request, response):
        """
        Service callback to enable/disable robot movement
        """
        self.movement_enabled = request.data
        status = "ENABLED" if self.movement_enabled else "DISABLED"
        self.get_logger().info(f'Robot movement: {status}')
        
        # Stop robot if disabling movement
        if not self.movement_enabled:
            cmd = Twist()
            self.publisher.publish(cmd)
        
        response.success = True
        response.message = f'Movement {status}'
        return response
    
    def change_direction_callback(self, request, response):
        """
        Service callback to change direction
        """
        self.forward_direction = request.data
        direction_str = "FORWARD (front camera)" if self.forward_direction else "BACKWARD (rear camera)"
        self.get_logger().info(f'Direction changed to: {direction_str}')
        
        # Reset PID when changing direction
        self.last_error = 0.0
        self.integral = 0.0
        self.last_detected_aruco = None
        
        response.success = True
        response.message = f'Direction set to {direction_str}'
        return response
    
    def front_camera_callback(self, data):
        """
        Callback for front camera
        """
        if self.forward_direction:
            self.process_image(data, "FRONT")
    
    def rear_camera_callback(self, data):
        """
        Callback for rear camera
        """
        if not self.forward_direction:
            self.process_image(data, "REAR")

    def get_contour_data(self, mask):
        """
        Return the centroid and orientation of the largest contour in the binary image 'mask' (the line) 
        """
        # Constants
        MIN_AREA_TRACK = 50  # Minimum area for track marks

        # Get a list of contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        line = {}
        max_area = 0

        for contour in contours:
            M = cv2.moments(contour)

            if M['m00'] > MIN_AREA_TRACK:
                area = M['m00']
                if area > max_area:
                    max_area = area
                    # Centroid of the line
                    line['x'] = int(M["m10"]/M["m00"])
                    line['y'] = int(M["m01"]/M["m00"])
                    
                    # Calculate orientation using image moments
                    # This helps understand if the line is tilted
                    if len(contour) >= 5:  # Need at least 5 points for fitEllipse
                        try:
                            ellipse = cv2.fitEllipse(contour)
                            line['angle'] = ellipse[2]  # Angle in degrees
                        except:
                            line['angle'] = 90.0  # Default: vertical line
                    else:
                        line['angle'] = 90.0

        return line

    def process_image(self, data, camera_name):
        """
        Main image processing function for both cameras
        """
        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            # Get image dimensions
            height, width, _ = current_frame.shape
            
            # Define Region of Interest (ROI) - bottom portion of image only
            # Focus on the area closer to the robot
            roi_start_row = int(height * 0.4)  # Start at 40% down from top (bottom 60%)
            roi_frame = current_frame[roi_start_row:height, 0:width]
            
            # Detect ArUco markers ONLY in the ROI region
            gray_roi = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray_roi)
            
            # Display detected ArUco markers
            if ids is not None and len(ids) > 0:
                # Draw detected markers on ROI
                cv2.aruco.drawDetectedMarkers(roi_frame, corners, ids)
                
                # Check if this is a new detection (avoid spam)
                current_time = time.time()
                for marker_id in ids.flatten():
                    if marker_id <= 7:  # IDs 0-7
                        if self.last_detected_aruco != marker_id or (current_time - self.last_aruco_time) > 3.0:
                            self.get_logger().info(f'======> Detected ArUco Marker: {marker_id} <=======')
                            self.last_detected_aruco = marker_id
                            self.last_aruco_time = current_time
            
            # Convert BGR to HSV
            hsv_image = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)

            # Define range of blue color in HSV
            lower_blue = np.array([100, 50, 50])   # Lower bound of blue color
            upper_blue = np.array([130, 255, 255])  # Upper bound of blue color

            # Create a binary mask
            blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

            # Apply the mask to the ROI image
            blue_segmented_image = cv2.bitwise_and(roi_frame, roi_frame, mask=blue_mask)

            # Detect line and get its centroid
            line = self.get_contour_data(blue_mask)

            # Move depending on detection 
            cmd = Twist()
            roi_height, roi_width, _ = blue_segmented_image.shape
            
            # Calculate time delta for PID
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            if line:
                x = line['x']
                error = x - roi_width//2
                
                # Calculate angle error (line should be vertical, around 90 degrees)
                # If angle is less than 90, line is tilted left; if more than 90, tilted right
                angle = line.get('angle', 90.0)
                angle_error = angle - 90.0  # Positive if tilted right, negative if tilted left
                
                # PID calculation for position error
                # Proportional term
                P = KP * error
                
                # Integral term (accumulate error over time)
                self.integral += error * dt
                # Prevent integral windup
                self.integral = max(-100, min(100, self.integral))
                I = KI * self.integral
                
                # Derivative term (rate of change of error)
                derivative = (error - self.last_error) / dt if dt > 0 else 0
                D = KD * derivative
                
                # Update last error
                self.last_error = error
                
                # Calculate angular velocity with PID + angle correction
                # Position correction + Orientation correction
                angular_z = -(P + I + D) - (KP_ANGLE * angle_error)
                
                # Limit angular velocity to maximum
                angular_z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_z))
                
                # Slow down in sharp turns (when error is large)
                if abs(error) > roi_width * 0.3:  # If line is more than 30% off-center
                    cmd.linear.x = LINEAR_SPEED * 0.6  # Reduce speed to 60%
                else:
                    cmd.linear.x = LINEAR_SPEED
                
                # Invert linear velocity if going backward
                if not self.forward_direction:
                    cmd.linear.x = -cmd.linear.x
                
                cmd.angular.z = angular_z
                
                # Draw circle on detected line for visualization
                cv2.circle(blue_segmented_image, (line['x'], line['y']), 5, (0, 0, 255), 7)
                # Draw center line reference
                cv2.line(blue_segmented_image, (roi_width//2, 0), (roi_width//2, roi_height), (0, 255, 0), 2)
                
                # Display camera name and direction
                direction_text = f"{camera_name} - {'FORWARD' if self.forward_direction else 'BACKWARD'}"
                cv2.putText(blue_segmented_image, direction_text, 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                
                # Display PID values and angle
                cv2.putText(blue_segmented_image, f"P:{P:.2f} I:{I:.2f} D:{D:.2f}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(blue_segmented_image, f"Angle:{angle:.1f}° Error:{error:.0f}px", 
                           (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # Display last detected ArUco if any
                if self.last_detected_aruco is not None:
                    cv2.putText(blue_segmented_image, f"ArUco: {self.last_detected_aruco}", 
                               (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # No line detected, stop
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.integral = 0.0  # Reset integral when line is lost
                self.get_logger().warn('No blue line detected!', throttle_duration_sec=1.0)
            
            # Log the error and angular velocity (throttled to avoid spam)
            if line:
                self.get_logger().debug(f"Error: {error} | Angular Z: {cmd.angular.z}")

            # Send the command to execute ONLY if movement is enabled AND no obstacle
            if self.movement_enabled:
                # Check for obstacles in the direction of travel
                if self.obstacle_detected:
                    # STOP! Obstacle detected
                    stop_cmd = Twist()
                    self.publisher.publish(stop_cmd)
                else:
                    # Safe to move
                    self.publisher.publish(cmd)
            else:
                # Ensure robot is stopped when movement is disabled
                stop_cmd = Twist()
                self.publisher.publish(stop_cmd)
            
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
