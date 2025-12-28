#!/usr/bin/env python3
"""
Mapping avec scan 360° puis wall-following
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math

class WallFollowerMapper(Node):
    def __init__(self):
        super().__init__('wall_follower_mapper')
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Paramètres - VITESSES RÉDUITES pour mapping propre
        self.linear_speed = 0.08  # m/s - Plus lent pour éviter artifacts
        self.angular_speed = 0.2  # rad/s - Rotation plus lente
        self.wall_distance = 0.6  # Distance cible du mur
        self.safety_distance = 0.5  # Distance de sécurité augmentée
        
        # État du scan
        self.scan_data = None
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        self.get_logger().info('╔═══════════════════════════════════════════════════╗')
        self.get_logger().info('║   WALL FOLLOWER MAPPER - Scan 360° + Mur         ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════╝')
        time.sleep(2)
        
    def scan_callback(self, msg):
        """Traiter les données du LIDAR"""
        self.scan_data = msg
        ranges = msg.ranges
        n = len(ranges)
        
        if n == 0:
            return
        
        # Découper en 3 zones: droite (0-120°), avant (120-240°), gauche (240-360°)
        right_sector = ranges[0:n//3]
        front_sector = ranges[n//3:2*n//3]
        left_sector = ranges[2*n//3:n]
        
        # Filtrer les valeurs infinies
        def get_min_valid(sector):
            valid = [r for r in sector if not math.isinf(r) and r > 0.1]
            return min(valid) if valid else float('inf')
        
        self.right_distance = get_min_valid(right_sector)
        self.front_distance = get_min_valid(front_sector)
        self.left_distance = get_min_valid(left_sector)
        
    def stop(self):
        """Arrêter le robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        
    def scan_360(self, duration=30):
        """Scan complet 360° sur place"""
        self.get_logger().info(f'🔍 SCAN 360° COMPLET ({duration}s)')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.15  # Rotation très lente pour scan propre
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop()
        self.get_logger().info('✅ Scan 360° terminé')
        
    def pause(self, duration=5):
        """Pause pour laisser SLAM traiter"""
        self.get_logger().info(f'⏸️  Pause {duration}s pour SLAM...')
        self.stop()
        time.sleep(duration)
        
    def follow_wall_timed(self, duration):
        """Suivre le mur pendant une durée"""
        self.get_logger().info(f'🧱 SUIVI DU MUR pendant {duration}s')
        
        start_time = time.time()
        iteration = 0
        
        while time.time() - start_time < duration:
            iteration += 1
            rclpy.spin_once(self, timeout_sec=0.05)
            
            if self.scan_data is None:
                continue
            
            msg = Twist()
            
            # Log tous les 50 iterations
            if iteration % 50 == 0:
                self.get_logger().info(
                    f'Distances - Avant: {self.front_distance:.2f}m, '
                    f'Gauche: {self.left_distance:.2f}m, '
                    f'Droite: {self.right_distance:.2f}m'
                )
            
            # Obstacle devant - tourner à gauche
            if self.front_distance < self.safety_distance:
                self.get_logger().info('⚠️  Obstacle devant - Tourner à gauche')
                msg.linear.x = 0.0
                msg.angular.z = self.angular_speed
                
            # Mur à gauche trop proche - s'éloigner
            elif self.left_distance < self.wall_distance - 0.15:
                msg.linear.x = self.linear_speed * 0.6
                msg.angular.z = -self.angular_speed * 0.3  # Légèrement à droite
                
            # Mur à gauche trop loin - se rapprocher
            elif self.left_distance > self.wall_distance + 0.15:
                msg.linear.x = self.linear_speed * 0.6
                msg.angular.z = self.angular_speed * 0.3  # Légèrement à gauche
                
            # Bon alignement - avancer droit
            else:
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
            
            self.cmd_vel_pub.publish(msg)
        
        self.stop()
        self.get_logger().info('✅ Suivi du mur terminé')
        
    def run_mapping(self):
        """Stratégie de mapping: Scan 360° puis suivi de mur"""
        self.get_logger().info('')
        self.get_logger().info('🚀 DÉBUT DU MAPPING')
        self.pause(4)
        
        # ÉTAPE 1: Scan 360° initial
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info('ÉTAPE 1: SCAN 360° INITIAL')
        self.get_logger().info('═' * 50)
        self.scan_360(30)
        self.pause(4)
        
        # ÉTAPE 2: Trouver le mur le plus proche
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info('ÉTAPE 2: POSITIONNEMENT VERS LE MUR')
        self.get_logger().info('═' * 50)
        
        # Attendre les données du scan
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan_data is not None:
                break
        
        # Tourner pour avoir le mur à gauche
        self.get_logger().info('🔄 Orientation pour avoir le mur à gauche')
        msg = Twist()
        msg.angular.z = self.angular_speed
        for _ in range(40):  # ~90 degrés
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()
        self.pause(2)
        
        # ÉTAPE 3: Suivre le mur - Premier tour complet
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info('ÉTAPE 3: PREMIER TOUR COMPLET (suivre le mur)')
        self.get_logger().info('═' * 50)
        self.follow_wall_timed(150)  # 2.5 minutes - plus lent = plus propre
        self.pause(8)
        
        # ÉTAPE 4: Scan 360° après le tour
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info('ÉTAPE 4: SCAN 360° INTERMÉDIAIRE')
        self.get_logger().info('═' * 50)
        self.scan_360(30)
        self.pause(4)
        
        # ÉTAPE 5: Deuxième tour pour compléter
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info('ÉTAPE 5: DEUXIÈME TOUR (compléter la carte)')
        self.get_logger().info('═' * 50)
        self.follow_wall_timed(150)  # 2.5 minutes - plus lent = plus propre
        self.pause(8)
        
        # ÉTAPE 6: Scan final
        self.get_logger().info('')
        self.get_logger().info('═' * 50)
        self.get_logger().info('ÉTAPE 6: SCAN 360° FINAL')
        self.get_logger().info('═' * 50)
        self.scan_360(30)
        self.pause(5)
        
        self.stop()
        
        self.get_logger().info('')
        self.get_logger().info('╔═══════════════════════════════════════════════════╗')
        self.get_logger().info('║     ✅ MAPPING TERMINÉ AVEC SUCCÈS !              ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info('💾 Pour sauvegarder la carte:')
        self.get_logger().info('   ros2 run nav2_map_server map_saver_cli -f ~/my_robot_map')

def main(args=None):
    rclpy.init(args=args)
    mapper = WallFollowerMapper()
    
    try:
        mapper.run_mapping()
    except KeyboardInterrupt:
        mapper.get_logger().info('⚠️  Arrêt demandé')
    finally:
        mapper.stop()
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
