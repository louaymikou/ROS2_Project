#!/usr/bin/env python3
"""
Script de mapping DÉTAILLÉ - passe près de chaque obstacle
Parcours optimisé pour détecter packages et étagère
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class DetailedMapper(Node):
    def __init__(self):
        super().__init__('detailed_mapper')
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
        # Vitesses TRÈS LENTES pour précision maximale
        self.linear_speed = 0.06  # Très lent
        self.angular_speed = 0.12  # Rotation très lente
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║   DETAILED MAPPER - DÉTECTION PRÉCISE DES OBSTACLES     ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        time.sleep(2)
        
    def stop(self):
        """Arrêter le robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(5):  # Publier plusieurs fois pour s'assurer
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        
    def move_forward(self, duration):
        """Avancer pendant une durée"""
        self.get_logger().info(f'⬆️  Avancer {duration}s...')
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = 0.0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        
        self.stop()
        
    def move_backward(self, duration):
        """Reculer pendant une durée"""
        self.get_logger().info(f'⬇️  Reculer {duration}s...')
        msg = Twist()
        msg.linear.x = -self.linear_speed
        msg.angular.z = 0.0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        
        self.stop()
        
    def rotate(self, duration, direction='left'):
        """Tourner sur place"""
        angle = "gauche" if direction == 'left' else "droite"
        self.get_logger().info(f'🔄 Rotation {angle} {duration}s...')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.angular_speed if direction == 'left' else -self.angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        
        self.stop()
        
    def scan_360(self, duration=25):
        """Scan complet 360°"""
        self.get_logger().info(f'🔍 Scan 360° ({duration}s)...')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        
        self.stop()
        
    def pause(self, duration=4):
        """Pause pour SLAM"""
        self.get_logger().info(f'⏸️  Pause {duration}s...')
        self.stop()
        time.sleep(duration)
        
    def run_mapping_sequence(self):
        """Parcours DÉTAILLÉ pour détecter tous les obstacles"""
        self.get_logger().info('🚀 DÉBUT DU MAPPING DÉTAILLÉ')
        self.pause(4)
        
        # ========== PHASE 1: Scan initial au centre (0,0) ==========
        self.get_logger().info('📍 PHASE 1: Scan 360° au point de départ')
        self.scan_360(25)
        self.pause(5)
        
        # ========== PHASE 2: Aller vers les PACKAGES (Nord) ==========
        self.get_logger().info('📍 PHASE 2: Aller vers la zone des packages (Nord)')
        self.move_forward(12)  # ~0.72m vers le nord
        self.pause(4)
        
        # Scan pour voir les packages au nord
        self.get_logger().info('📍 PHASE 3: Scan des packages - premier scan')
        self.scan_360(25)
        self.pause(4)
        
        # ========== PHASE 4: Aller vers les PETITS packages (Nord-Ouest) ==========
        self.get_logger().info('📍 PHASE 4: Vers petits packages (Nord-Ouest)')
        self.rotate(13, 'left')  # ~90° gauche
        self.pause(3)
        self.move_forward(10)  # Vers x=-3
        self.pause(3)
        
        # Scanner autour des petits packages
        self.get_logger().info('📍 PHASE 5: Scanner les petits packages bleus')
        self.scan_360(25)
        self.pause(4)
        
        # ========== PHASE 6: Vers les GRANDS packages (Est) ==========
        self.get_logger().info('📍 PHASE 6: Vers grands packages (Est)')
        self.rotate(13, 'right')  # ~90° droite
        self.pause(3)
        self.move_forward(15)  # Vers x=0 puis x=2
        self.pause(3)
        
        # Scanner autour des grands packages
        self.get_logger().info('📍 PHASE 7: Scanner les grands packages orange')
        self.scan_360(25)
        self.pause(4)
        
        # ========== PHASE 8: Explorer le mur NORD ==========
        self.get_logger().info('📍 PHASE 8: Longer le mur Nord')
        self.move_forward(10)  # Continuer vers l'est
        self.pause(3)
        self.rotate(13, 'right')  # Face au nord
        self.pause(2)
        self.move_forward(8)  # Vers le mur nord
        self.pause(3)
        
        # Scan le long du mur nord
        self.get_logger().info('📍 PHASE 9: Scan le long du mur Nord')
        self.scan_360(20)
        self.pause(4)
        
        # ========== PHASE 10: Coin Nord-Est ==========
        self.get_logger().info('📍 PHASE 10: Coin Nord-Est')
        self.rotate(13, 'right')  # Face à l'est
        self.pause(2)
        self.move_forward(10)  # Vers le coin NE
        self.pause(3)
        self.scan_360(20)
        self.pause(4)
        
        # ========== PHASE 11: Longer mur EST vers ÉTAGÈRE ==========
        self.get_logger().info('📍 PHASE 11: Descendre vers étagère (Sud)')
        self.rotate(13, 'right')  # Face au sud
        self.pause(2)
        self.move_forward(20)  # Descendre vers y=-4
        self.pause(4)
        
        # ========== PHASE 12: Scanner l'ÉTAGÈRE ==========
        self.get_logger().info('📍 PHASE 12: Scanner l\'étagère de stockage')
        self.scan_360(30)  # Scan plus long pour l'étagère
        self.pause(5)
        
        # S'approcher de l'étagère
        self.get_logger().info('📍 PHASE 13: S\'approcher de l\'étagère')
        self.rotate(13, 'right')  # Face à l'ouest (vers étagère)
        self.pause(2)
        self.move_forward(5)  # S'approcher
        self.pause(3)
        self.scan_360(25)
        self.pause(4)
        
        # ========== PHASE 14: Coin Sud-Est ==========
        self.get_logger().info('📍 PHASE 14: Coin Sud-Est')
        self.rotate(13, 'left')  # Face au sud
        self.pause(2)
        self.move_forward(12)  # Vers coin SE
        self.pause(3)
        self.scan_360(20)
        self.pause(4)
        
        # ========== PHASE 15: Longer mur SUD ==========
        self.get_logger().info('📍 PHASE 15: Longer le mur Sud')
        self.rotate(13, 'right')  # Face à l'ouest
        self.pause(2)
        self.move_forward(25)  # Traverser tout le sud
        self.pause(4)
        self.scan_360(20)
        self.pause(4)
        
        # ========== PHASE 16: Coin Sud-Ouest ==========
        self.get_logger().info('📍 PHASE 16: Coin Sud-Ouest')
        self.move_forward(10)
        self.pause(3)
        self.scan_360(20)
        self.pause(4)
        
        # ========== PHASE 17: Longer mur OUEST ==========
        self.get_logger().info('📍 PHASE 17: Remonter mur Ouest')
        self.rotate(13, 'right')  # Face au nord
        self.pause(2)
        self.move_forward(25)  # Remonter
        self.pause(4)
        self.scan_360(20)
        self.pause(4)
        
        # ========== PHASE 18: Retour centre et scan final ==========
        self.get_logger().info('📍 PHASE 18: Retour au centre')
        self.rotate(13, 'right')  # Face à l'est
        self.pause(2)
        self.move_forward(15)  # Vers centre
        self.pause(3)
        self.rotate(13, 'right')  # Face au sud
        self.pause(2)
        self.move_forward(12)  # Vers centre
        self.pause(4)
        
        # Scan final
        self.get_logger().info('📍 PHASE 19: Scan final 360° au centre')
        self.scan_360(30)
        self.pause(5)
        
        self.stop()
        
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║     ✅ MAPPING DÉTAILLÉ TERMINÉ !                        ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info('💾 Sauvegarder avec:')
        self.get_logger().info('   ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map')

def main(args=None):
    rclpy.init(args=args)
    mapper = DetailedMapper()
    
    try:
        mapper.run_mapping_sequence()
    except KeyboardInterrupt:
        mapper.get_logger().info('⚠️  Arrêt demandé')
    finally:
        mapper.stop()
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
