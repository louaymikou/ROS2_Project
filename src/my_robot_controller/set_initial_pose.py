#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        
        # Publisher pour la pose initiale
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.get_logger().info('✅ Nœud de pose initiale créé')
        
    def set_initial_pose(self, x=0.0, y=0.0, yaw=0.0):
        """Définir la pose initiale du robot dans la carte"""
        import math
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion depuis yaw)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance (incertitude)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25  # x
        msg.pose.covariance[7] = 0.25  # y
        msg.pose.covariance[35] = 0.06853  # yaw
        
        self.get_logger().info('')
        self.get_logger().info('═' * 60)
        self.get_logger().info('📍 Définition de la pose initiale du robot')
        self.get_logger().info(f'   Position: x={x:.2f}m, y={y:.2f}m')
        self.get_logger().info(f'   Orientation: yaw={yaw:.2f}rad ({math.degrees(yaw):.1f}°)')
        self.get_logger().info('═' * 60)
        
        # Publier plusieurs fois pour s'assurer que AMCL reçoit le message
        for i in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.1)
        
        self.get_logger().info('✅ Pose initiale publiée !')
        self.get_logger().info('   Attendez 2-3 secondes que AMCL se localise...')

def main():
    rclpy.init()
    
    node = SetInitialPose()
    
    # Attendre un peu que tout soit prêt
    time.sleep(1)
    
    # Définir la pose initiale au centre (0, 0)
    # Ajustez ces valeurs selon où se trouve votre robot dans Gazebo
    node.set_initial_pose(x=0.0, y=0.0, yaw=0.0)
    
    time.sleep(2)
    
    node.get_logger().info('')
    node.get_logger().info('✅ Robot localisé ! Vous pouvez maintenant naviguer.')
    node.get_logger().info('   Lancez: python3 src/my_robot_controller/navigate_to_package.py')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
