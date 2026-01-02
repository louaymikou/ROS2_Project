#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# Constants
LINEAR_SPEED = 0.25  # Vitesse linéaire constante
KP = 1.5/100  # Constante proportionnelle pour le contrôle de rotation
MIN_AREA_TRACK = 500  # Surface minimale pour détecter la ligne

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Bridge pour convertir les images ROS en OpenCV
        self.bridge = CvBridge()
        
        # Subscriber pour la caméra
        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher pour contrôler le robot
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            10
        )
        
        # Paramètres ajustables
        self.declare_parameter('linear_speed', LINEAR_SPEED)
        self.declare_parameter('kp', KP)
        self.declare_parameter('min_area', MIN_AREA_TRACK)
        self.declare_parameter('debug_visual', False)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp').value
        self.min_area = self.get_parameter('min_area').value
        self.debug_visual = self.get_parameter('debug_visual').value
        
        # État du suivi
        self.following = True
        
        self.get_logger().info('Line Follower démarré avec algorithme optimisé!')
        self.get_logger().info(f'Vitesse: {self.linear_speed} | Kp: {self.kp} | MinArea: {self.min_area}')
    
    def get_contour_data(self, mask):
        """
        Retourne le centroïde du plus grand contour dans le masque binaire (la ligne)
        """
        # Obtenir la liste des contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        line = {}
        
        for contour in contours:
            M = cv2.moments(contour)
            
            if M['m00'] > self.min_area:
                # Le contour fait partie de la ligne
                line['x'] = int(M["m10"] / M["m00"])
                line['y'] = int(M["m01"] / M["m00"])
                line['area'] = M['m00']
        
        return line
    
    def image_callback(self, msg):
        """Traite l'image de la caméra et contrôle le robot"""
        if not self.following:
            return
        
        try:
            # Convertir l'image ROS en image OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            
            # Convertir BGR en HSV pour une meilleure détection des couleurs
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Définir la plage de couleur pour détecter la ligne NOIRE sur fond gris/blanc
            # Pour une ligne noire sur fond clair, on utilise les valeurs basses de V (Value)
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 80])  # Seuil de luminosité pour le noir
            
            # Créer un masque binaire
            black_mask = cv2.inRange(hsv_image, lower_black, upper_black)
            
            # Appliquer un flou gaussien pour réduire le bruit
            black_mask = cv2.GaussianBlur(black_mask, (5, 5), 0)
            
            # Détecter la ligne et obtenir son centroïde
            line = self.get_contour_data(black_mask)
            
            # Créer la commande de mouvement
            cmd = Twist()
            error = 0
            
            if line:
                x = line['x']
                
                # Calculer l'erreur (distance du centre de la ligne au centre de l'image)
                error = x - w // 2
                
                # Déplacer le robot
                cmd.linear.x = self.linear_speed
                
                # Déterminer la vitesse angulaire pour centrer la ligne dans la caméra
                cmd.angular.z = float(error) * -self.kp
                
                # Log pour le débogage
                self.get_logger().info(
                    f'Erreur: {error}px | Angular Z: {cmd.angular.z:.3f} | Aire: {int(line.get("area", 0))}',
                    throttle_duration_sec=0.5
                )
                
                # Debug visuel optionnel
                if self.debug_visual:
                    # Appliquer le masque à l'image originale
                    black_segmented = cv2.bitwise_and(cv_image, cv_image, mask=black_mask)
                    # Dessiner le centroïde
                    cv2.circle(black_segmented, (line['x'], line['y']), 5, (0, 0, 255), 7)
                    # Dessiner la ligne centrale
                    cv2.line(black_segmented, (w//2, 0), (w//2, h), (0, 255, 0), 2)
                    cv2.imshow("Line Detection", black_segmented)
                    cv2.imshow("Black Mask", black_mask)
                    cv2.waitKey(1)
            else:
                # Ligne non détectée, arrêter le robot
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().warn('Ligne non détectée!', throttle_duration_sec=1.0)
            
            # Envoyer la commande
            self.cmd_vel_pub.publish(cmd)
        
        except Exception as e:
            self.get_logger().error(f'Erreur traitement image: {str(e)}')
    
    def start_following(self):
        """Démarre le suivi de ligne"""
        self.following = True
        self.get_logger().info('Suivi de ligne ACTIVÉ')
    
    def stop_following(self):
        """Arrête le suivi de ligne"""
        self.following = False
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Suivi de ligne DÉSACTIVÉ')

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    
    # Démarrer le suivi automatiquement
    node.start_following()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_following()
        node.destroy_node()
        if node.debug_visual:
            cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
