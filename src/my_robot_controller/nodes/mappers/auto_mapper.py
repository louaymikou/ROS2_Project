#!/usr/bin/env python3
"""
Script automatique pour créer une carte SLAM
Le robot fait un parcours complet de l'environnement
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
        # Vitesses TRÈS LENTES pour éviter glissement
        self.linear_speed = 0.08  # Très très lent
        self.angular_speed = 0.15   # Rotation très lente
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║     AUTO MAPPER - CRÉATION AUTOMATIQUE DE CARTE          ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        time.sleep(2)
        
    def stop(self):
        """Arrêter le robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        
    def move_forward(self, duration):
        """Avancer pendant une durée"""
        self.get_logger().info(f'⬆️  Avancer pendant {duration}s...')
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = 0.0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        
        self.stop()
        
    def rotate(self, duration, direction='left'):
        """Tourner sur place"""
        angle = "gauche" if direction == 'left' else "droite"
        self.get_logger().info(f'🔄 Rotation {angle} pendant {duration}s...')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.angular_speed if direction == 'left' else -self.angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        
        self.stop()
        
    def pause(self, duration=3):
        """Pause pour laisser SLAM se stabiliser"""
        self.get_logger().info(f'⏸️  Pause {duration}s pour SLAM...')
        self.stop()
        time.sleep(duration)
        
    def run_mapping_sequence(self):
        """Exécuter la séquence de mapping complète"""
        self.get_logger().info('🚀 DÉBUT DU MAPPING AUTOMATIQUE')
        self.pause(3)
        
        # Tour complet de la pièce - DISTANCES AUGMENTÉES
        self.get_logger().info('📍 PHASE 1: Aller vers le Nord')
        self.move_forward(6.0)  # Augmenté pour atteindre le coin
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 2: Tourner à droite (90°)')
        self.rotate(5.0, 'right')
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 3: Longer le mur Est')
        self.move_forward(10.0)  # Augmenté pour tout le mur
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 4: Tourner à droite (90°)')
        self.rotate(5.0, 'right')
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 5: Longer le mur Sud')
        self.move_forward(10.0)  # Augmenté pour tout le mur
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 6: Tourner à droite (90°)')
        self.rotate(5.0, 'right')
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 7: Longer le mur Ouest')
        self.move_forward(10.0)  # Augmenté pour tout le mur
        self.pause(3)
        
        self.get_logger().info('📍 PHASE 8: Tourner à droite (90°)')
        self.rotate(5.0, 'right')
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 9: Retour vers le centre')
        self.move_forward(4.0)
        self.pause(4)
        
        # Rotation 360° pour bien scanner le centre
        self.get_logger().info('📍 PHASE 10: Scan 360° au centre')
        self.rotate(20.0, 'left')
        self.pause(5)
        
        # Explorer vers l\'étagère
        self.get_logger().info('📍 PHASE 11: Aller vers l\'étagère')
        self.rotate(5.0, 'right')
        self.pause(3)
        self.move_forward(6.0)  # Distance augmentée
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 12: Scanner l\'étagère')
        self.rotate(12.0, 'left')  # Rotation plus complète
        self.pause(4)
        
        self.get_logger().info('📍 PHASE 13: Retour au point de départ')
        self.rotate(10.0, 'left')
        self.move_forward(6.0)
        self.pause(4)
        
        self.stop()
        
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║     ✅ MAPPING TERMINÉ !                                  ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info('💾 Pour sauvegarder la carte, dans un nouveau terminal :')
        self.get_logger().info('   cd ~/ROS2_Project')
        self.get_logger().info('   source install/setup.bash')
        self.get_logger().info('   ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map')

def main(args=None):
    rclpy.init(args=args)
    mapper = AutoMapper()
    
    try:
        mapper.run_mapping_sequence()
    except KeyboardInterrupt:
        mapper.get_logger().info('⚠️  Arrêt demandé par l\'utilisateur')
    finally:
        mapper.stop()
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
