#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SimpleCameraView(Node):
    def __init__(self):
        super().__init__('simple_camera_view')
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Visualisation simple de la caméra - Appuyez sur Q pour quitter')
        self.get_logger().info('Vérifiez que vous voyez le SOL avec la LIGNE NOIRE')
    
    def image_callback(self, msg):
        try:
            # Convertir et afficher
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            h, w = cv_image.shape[:2]
            
            # Ajouter un repère au centre
            cv2.line(cv_image, (w//2, 0), (w//2, h), (0, 255, 0), 2)
            cv2.line(cv_image, (0, h//2), (w, h//2), (0, 255, 0), 2)
            cv2.circle(cv_image, (w//2, h//2), 10, (0, 0, 255), -1)
            
            # Afficher ce que voit la caméra
            cv2.imshow('VUE CAMERA - Le sol devrait etre visible ici', cv_image)
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                rclpy.shutdown()
        
        except Exception as e:
            self.get_logger().error(f'Erreur: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCameraView()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
