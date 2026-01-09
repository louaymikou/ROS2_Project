#!/usr/bin/env python3
"""
Script de comparaison visuelle entre l'odométrie des roues et l'odométrie fusionnée (EKF).
Affiche en temps réel les différences d'orientation et de position.

Usage:
    ros2 run my_robot_controller compare_odometry.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math


def quaternion_to_euler(q: Quaternion):
    """Convertit un quaternion en angles d'Euler (roll, pitch, yaw)"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class OdometryComparator(Node):
    def __init__(self):
        super().__init__('odometry_comparator')
        
        # Subscriber pour l'odométrie des roues
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/diff_cont/odom',
            self.wheel_odom_callback,
            10
        )
        
        # Subscriber pour l'odométrie fusionnée
        self.fused_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/local',
            self.fused_odom_callback,
            10
        )
        
        # Données
        self.wheel_odom = None
        self.fused_odom = None
        
        # Timer pour affichage périodique
        self.timer = self.create_timer(1.0, self.display_comparison)
        
        self.get_logger().info('🔍 Comparateur d\'odométrie démarré')
        self.get_logger().info('📊 Comparaison: /diff_cont/odom vs /odometry/local')
        self.get_logger().info('')
    
    def wheel_odom_callback(self, msg: Odometry):
        """Callback pour l'odométrie des roues"""
        self.wheel_odom = msg
    
    def fused_odom_callback(self, msg: Odometry):
        """Callback pour l'odométrie fusionnée"""
        self.fused_odom = msg
    
    def display_comparison(self):
        """Affiche la comparaison entre les deux odométries"""
        if self.wheel_odom is None or self.fused_odom is None:
            self.get_logger().warn('⏳ En attente des données d\'odométrie...')
            return
        
        # Extraire les positions
        wheel_pos = self.wheel_odom.pose.pose.position
        fused_pos = self.fused_odom.pose.pose.position
        
        # Extraire les orientations
        wheel_quat = self.wheel_odom.pose.pose.orientation
        fused_quat = self.fused_odom.pose.pose.orientation
        
        # Convertir en angles d'Euler
        wheel_roll, wheel_pitch, wheel_yaw = quaternion_to_euler(wheel_quat)
        fused_roll, fused_pitch, fused_yaw = quaternion_to_euler(fused_quat)
        
        # Calculer les différences
        diff_x = fused_pos.x - wheel_pos.x
        diff_y = fused_pos.y - wheel_pos.y
        diff_z = fused_pos.z - wheel_pos.z
        diff_distance = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
        
        diff_roll = math.degrees(fused_roll - wheel_roll)
        diff_pitch = math.degrees(fused_pitch - wheel_pitch)
        diff_yaw = math.degrees(fused_yaw - wheel_yaw)
        
        # Normaliser l'angle yaw entre -180 et 180
        if diff_yaw > 180:
            diff_yaw -= 360
        elif diff_yaw < -180:
            diff_yaw += 360
        
        # Affichage avec émojis et couleurs
        print('\n' + '='*70)
        print('📊 COMPARAISON ODOMÉTRIE')
        print('='*70)
        
        print('\n🌍 POSITION:')
        print(f'  Roues (wheel):  x={wheel_pos.x:7.3f}  y={wheel_pos.y:7.3f}  z={wheel_pos.z:7.3f}')
        print(f'  Fusionné (EKF): x={fused_pos.x:7.3f}  y={fused_pos.y:7.3f}  z={fused_pos.z:7.3f}')
        print(f'  📏 Différence:  Δx={diff_x:7.4f}  Δy={diff_y:7.4f}  Δz={diff_z:7.4f}')
        print(f'  📐 Distance:    {diff_distance:7.4f} m')
        
        print('\n🧭 ORIENTATION (degrés):')
        print(f'  Roues (wheel):  roll={math.degrees(wheel_roll):7.2f}°  pitch={math.degrees(wheel_pitch):7.2f}°  yaw={math.degrees(wheel_yaw):7.2f}°')
        print(f'  Fusionné (EKF): roll={math.degrees(fused_roll):7.2f}°  pitch={math.degrees(fused_pitch):7.2f}°  yaw={math.degrees(fused_yaw):7.2f}°')
        print(f'  🔄 Différence:  Δroll={diff_roll:7.2f}°  Δpitch={diff_pitch:7.2f}°  Δyaw={diff_yaw:7.2f}°')
        
        # Indicateurs de qualité
        print('\n✨ ÉVALUATION:')
        
        # Distance
        if diff_distance < 0.01:
            distance_status = '✅ Excellente'
        elif diff_distance < 0.05:
            distance_status = '✅ Bonne'
        elif diff_distance < 0.10:
            distance_status = '⚠️  Acceptable'
        else:
            distance_status = '❌ Importante'
        print(f'  Position:    {distance_status} (différence: {diff_distance:.4f} m)')
        
        # Orientation (principalement yaw)
        abs_diff_yaw = abs(diff_yaw)
        if abs_diff_yaw < 1.0:
            orientation_status = '✅ Excellente'
        elif abs_diff_yaw < 3.0:
            orientation_status = '✅ Bonne'
        elif abs_diff_yaw < 5.0:
            orientation_status = '⚠️  Acceptable'
        else:
            orientation_status = '❌ Importante'
        print(f'  Orientation: {orientation_status} (différence yaw: {abs_diff_yaw:.2f}°)')
        
        # Covariance de l'odométrie fusionnée (indicateur de confiance)
        fused_cov = self.fused_odom.pose.covariance
        position_uncertainty = math.sqrt(fused_cov[0] + fused_cov[7])  # x + y
        orientation_uncertainty = math.sqrt(fused_cov[35])  # yaw
        
        print('\n📊 CONFIANCE EKF:')
        print(f'  Incertitude position:    {position_uncertainty:.6f}')
        print(f'  Incertitude orientation: {orientation_uncertainty:.6f}')
        
        if position_uncertainty < 0.01:
            print('  ✅ L\'EKF est très confiant en la position')
        elif position_uncertainty < 0.1:
            print('  ✅ L\'EKF est confiant en la position')
        else:
            print('  ⚠️  L\'EKF a de l\'incertitude sur la position')
        
        print('='*70)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryComparator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n👋 Arrêt du comparateur d\'odométrie')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
