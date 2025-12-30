#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import time

class NavigateToPackage(Node):
    def __init__(self):
        super().__init__('navigate_to_package')
        
        # Client d'action Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('🚀 Navigation vers Package initialisée')
        self.get_logger().info('⏳ Attente du serveur Nav2...')
        
        # Attendre que le serveur soit prêt
        self._action_client.wait_for_server()
        self.get_logger().info('✅ Serveur Nav2 prêt !')
        
    def create_goal(self, x, y, yaw=0.0):
        """Créer un goal de navigation"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion depuis yaw)
        import math
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal_msg
    
    def navigate_to(self, x, y, yaw=0.0, package_name=""):
        """Envoyer un goal de navigation"""
        goal_msg = self.create_goal(x, y, yaw)
        
        self.get_logger().info('')
        self.get_logger().info('═' * 60)
        if package_name:
            self.get_logger().info(f'🎯 Navigation vers {package_name}')
        else:
            self.get_logger().info(f'🎯 Navigation vers position ({x:.2f}, {y:.2f})')
        self.get_logger().info(f'📍 Position: x={x:.2f}m, y={y:.2f}m, yaw={yaw:.2f}rad')
        self.get_logger().info('═' * 60)
        
        # Envoyer le goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejeté par Nav2 !')
            return False
        
        self.get_logger().info('✅ Goal accepté, navigation en cours...')
        
        # Attendre le résultat
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('')
            self.get_logger().info('🎉 Navigation réussie !')
            if package_name:
                self.get_logger().info(f'✅ Arrivé à {package_name} !')
            return True
        else:
            self.get_logger().error(f'❌ Navigation échouée (status: {status})')
            return False
    
    def feedback_callback(self, feedback_msg):
        """Callback pour le feedback de navigation"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        if distance > 0.1:  # Afficher seulement si distance significative
            self.get_logger().info(
                f'📊 Distance restante: {distance:.2f}m | '
                f'Temps: {feedback.navigation_time.sec}s'
            )

def main():
    rclpy.init()
    
    navigator = NavigateToPackage()
    
    # Attendre un peu pour que tout se stabilise
    navigator.get_logger().info('⏸️  Attente de 5 secondes pour stabilisation...')
    time.sleep(5)
    
    # Coordonnées du Package 1 (à ajuster selon votre monde)
    # Dans my_world.world, les packages sont généralement autour de (x=2, y=2)
    package1_x = 2.0
    package1_y = 2.0
    package1_yaw = 0.0
    
    # Naviguer vers Package 1
    success = navigator.navigate_to(
        package1_x, 
        package1_y, 
        package1_yaw,
        "Package 1"
    )
    
    if success:
        navigator.get_logger().info('')
        navigator.get_logger().info('╔' + '═' * 58 + '╗')
        navigator.get_logger().info('║' + ' ' * 15 + '✅ MISSION ACCOMPLIE !' + ' ' * 15 + '║')
        navigator.get_logger().info('╚' + '═' * 58 + '╝')
    else:
        navigator.get_logger().error('Mission échouée')
    
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
