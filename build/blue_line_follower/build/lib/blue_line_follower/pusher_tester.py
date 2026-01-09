#!/usr/bin/env python3
"""
Script de test pour vérifier le mécanisme de poussée
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class PusherTester(Node):
    def __init__(self):
        super().__init__('pusher_tester')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.pusher_position = None
        self.get_logger().info('🔍 Test du mécanisme de poussée démarré')
        self.get_logger().info('En attente des données joint_states...')

    def joint_state_callback(self, msg):
        try:
            # Trouver l'index du pusher_joint
            pusher_index = msg.name.index('pusher_joint')
            self.pusher_position = msg.position[pusher_index]
            
            self.get_logger().info(
                f'📍 Position du pousseur: {self.pusher_position:.4f} m',
                throttle_duration_sec=1.0  # Log toutes les 1 seconde
            )
            
            # Indiquer la zone
            if abs(self.pusher_position) < 0.05:
                zone = "CENTRE ⚪"
            elif self.pusher_position > 0.05:
                zone = "GAUCHE 🔵"
            else:
                zone = "DROITE 🔴"
            
            self.get_logger().info(
                f'🎯 Zone actuelle: {zone}',
                throttle_duration_sec=2.0
            )
            
        except (ValueError, IndexError):
            # pusher_joint n'est pas encore dans les messages
            pass


def main(args=None):
    rclpy.init(args=args)
    
    tester = PusherTester()
    
    print("\n" + "="*60)
    print("🤖 TEST DU MÉCANISME DE POUSSÉE")
    print("="*60)
    print("Ce script affiche la position en temps réel du pousseur.")
    print("Lancez le contrôleur dans un autre terminal pour tester.")
    print("\nPour lancer le contrôleur:")
    print("  ros2 run blue_line_follower pusher_controller")
    print("\nAppuyez sur Ctrl+C pour arrêter.")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Arrêt du test')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
