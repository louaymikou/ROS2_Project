#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class AlignedNavigator(Node):
    def __init__(self):
        super().__init__('aligned_navigator')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info('🤖 Aligned Navigator Ready!')
        
    def create_goal(self, x, y, yaw_degrees):
        """Crée un goal avec position (x,y) et orientation en degrés"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        
        # Convertir degrés en quaternion
        yaw_rad = math.radians(yaw_degrees)
        goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        return goal
    
    def send_goal(self, x, y, yaw_degrees, wait_time=15):
        """Envoie un goal et attend"""
        goal = self.create_goal(x, y, yaw_degrees)
        
        self.get_logger().info(f'📍 Envoi goal: x={x:.2f}, y={y:.2f}, yaw={yaw_degrees}°')
        self.publisher.publish(goal)
        
        self.get_logger().info(f'⏳ Attente {wait_time}s pour atteindre le goal...')
        time.sleep(wait_time)
    
    def navigate_aligned(self):
        """Navigation alignée en 3 étapes"""
        
        self.get_logger().info('🚀 Démarrage navigation alignée!')
        self.get_logger().info('='*50)
        
        # Étape 1: Aller à x=1.8 (ligne droite sur axe X)
        self.get_logger().info('📌 ÉTAPE 1: Ligne droite sur X jusqu\'à x=1.8')
        self.send_goal(x=1.8, y=0.0, yaw_degrees=0, wait_time=12)
        
        # Étape 2: Rotation 90° sur place
        self.get_logger().info('📌 ÉTAPE 2: Rotation 90° sur place')
        self.send_goal(x=1.8, y=0.0, yaw_degrees=90, wait_time=8)
        
        # Étape 3: Aller à y=4.7 (ligne droite sur axe Y)
        self.get_logger().info('📌 ÉTAPE 3: Ligne droite sur Y jusqu\'à y=4.7')
        self.send_goal(x=1.8, y=4.7, yaw_degrees=90, wait_time=15)
        
        self.get_logger().info('='*50)
        self.get_logger().info('✅ Navigation alignée terminée!')
        self.get_logger().info(f'🎯 Position finale: x=1.8, y=4.7, orientation=90°')

def main(args=None):
    rclpy.init(args=args)
    navigator = AlignedNavigator()
    
    try:
        # Attendre que Nav2 soit prêt
        navigator.get_logger().info('⏳ Attente 3s pour que Nav2 soit prêt...')
        time.sleep(3)
        
        # Lancer la navigation alignée
        navigator.navigate_aligned()
        
    except KeyboardInterrupt:
        navigator.get_logger().info('❌ Navigation interrompue')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
