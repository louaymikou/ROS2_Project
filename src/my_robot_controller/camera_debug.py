#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraDebug(Node):
    def __init__(self):
        super().__init__('camera_debug')
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Camera Debug démarré - Appuyez sur Q pour quitter')
    
    def image_callback(self, msg):
        try:
            # Convertir l'image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]
            
            # Convertir en niveaux de gris
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Afficher les valeurs min/max pour ajuster le seuil
            min_val = np.min(gray)
            max_val = np.max(gray)
            mean_val = np.mean(gray)
            
            # Appliquer le seuil
            threshold_value = 80
            _, binary = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
            binary = cv2.GaussianBlur(binary, (5, 5), 0)
            
            # Afficher les infos d'analyse
            cv2.putText(vis_image, f'Min: {int(min_val)} Max: {int(max_val)} Mean: {int(mean_val)}', 
                        (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(vis_image, f'Threshold: {threshold_value}', 
                        (10, h-35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # ROI
            roi_height = int(h * 0.4)
            roi = binary[h - roi_height:h, :]
            
            # Détecter les contours
            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Créer image de visualisation
            vis_image = cv_image.copy()
            
            # Dessiner la zone ROI
            cv2.rectangle(vis_image, (0, h - roi_height), (w, h), (0, 255, 0), 2)
            
            # Dessiner le centre de l'image
            cv2.line(vis_image, (w//2, 0), (w//2, h), (255, 0, 0), 2)
            
            # Afficher le nombre de contours détectés
            cv2.putText(vis_image, f'Contours: {len(contours)}', (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 500:
                    # Dessiner le contour
                    offset_contour = largest_contour.copy()
                    offset_contour[:, :, 1] += (h - roi_height)
                    cv2.drawContours(vis_image, [offset_contour], -1, (0, 255, 255), 3)
                    
                    # Calculer et dessiner le centre
                    M = cv2.moments(largest_contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00']) + (h - roi_height)
                        cv2.circle(vis_image, (cx, cy), 10, (0, 0, 255), -1)
                        
                        # Afficher l'erreur
                        error = cx - w // 2
                        cv2.putText(vis_image, f'Erreur: {error}px', (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(vis_image, f'Aire: {int(area)}', (10, 70),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Afficher les images
            cv2.imshow('Camera Original', cv_image)
            cv2.imshow('Masque Binaire', binary)
            cv2.imshow('Detection', vis_image)
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                rclpy.shutdown()
        
        except Exception as e:
            self.get_logger().error(f'Erreur: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraDebug()
    
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
