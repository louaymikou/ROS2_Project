#!/usr/bin/env python3
"""
Test de diagnostic pour la navigation ArUco
Affiche des informations détaillées sur l'état du système
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ArucoDiagnostic(Node):
    def __init__(self):
        super().__init__('aruco_diagnostic')
        
        self.bridge = CvBridge()
        
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Subscribers
        self.front_sub = self.create_subscription(
            Image, '/camera/image_raw', self.front_callback, 10)
        self.rear_sub = self.create_subscription(
            Image, '/rear_camera/image_raw', self.rear_callback, 10)
        
        self.get_logger().info('🔍 Diagnostic ArUco démarré')
        self.get_logger().info('   Affichage des détections en temps réel...')

    def front_callback(self, msg):
        self._process_camera(msg, "FRONT")

    def rear_callback(self, msg):
        self._process_camera(msg, "REAR")

    def _process_camera(self, msg, camera_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None and len(ids) > 0:
                detected_ids = [int(id[0]) for id in ids]
                self.get_logger().info(
                    f'📸 {camera_name}: ArUcos détectés {detected_ids}',
                    throttle_duration_sec=2.0
                )
            
        except Exception as e:
            self.get_logger().error(f'Erreur: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt du diagnostic')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
