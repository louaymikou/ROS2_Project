#!/usr/bin/env python3
"""
Test de l'odométrie - Vérifie que l'odom fonctionne correctement
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

class OdometryTester(Node):
    def __init__(self):
        super().__init__('odometry_tester')
        
        # Subscriber pour l'odométrie
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher pour bouger le robot
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            10
        )
        
        self.last_odom = None
        self.initial_odom = None
        self.odom_received = False
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║          TEST DE L\'ODOMÉTRIE                             ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
    def odom_callback(self, msg):
        """Callback pour recevoir l'odométrie"""
        if not self.odom_received:
            self.get_logger().info('✅ Topic /odom reçu !')
            self.odom_received = True
            self.initial_odom = msg
        
        self.last_odom = msg
        
    def get_position(self, odom_msg):
        """Extraire position (x, y) de l'odométrie"""
        return (odom_msg.pose.pose.position.x, 
                odom_msg.pose.pose.position.y)
    
    def get_yaw(self, odom_msg):
        """Extraire l'orientation (yaw) de l'odométrie"""
        q = odom_msg.pose.pose.orientation
        # Conversion quaternion -> yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def stop(self):
        """Arrêter le robot"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        
    def move_forward(self, duration=2.0):
        """Bouger en avant"""
        self.get_logger().info(f'⬆️  Test: Avancer {duration}s...')
        msg = Twist()
        msg.linear.x = 0.2
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        
        self.stop()
        
    def rotate(self, duration=2.0):
        """Tourner sur place"""
        self.get_logger().info(f'🔄 Test: Rotation {duration}s...')
        msg = Twist()
        msg.angular.z = 0.3
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        
        self.stop()
        
    def display_odom_info(self):
        """Afficher les infos d'odométrie"""
        if self.last_odom is None:
            self.get_logger().warn('❌ Pas de données d\'odométrie reçues')
            return False
            
        pos = self.get_position(self.last_odom)
        yaw = self.get_yaw(self.last_odom)
        vel = self.last_odom.twist.twist
        
        self.get_logger().info('┌────────────────────────────────────────────────┐')
        self.get_logger().info(f'│ Position (x, y): ({pos[0]:.3f}, {pos[1]:.3f})         ')
        self.get_logger().info(f'│ Orientation (yaw): {math.degrees(yaw):.1f}°           ')
        self.get_logger().info(f'│ Vitesse linéaire: {vel.linear.x:.3f} m/s      ')
        self.get_logger().info(f'│ Vitesse angulaire: {vel.angular.z:.3f} rad/s  ')
        self.get_logger().info('└────────────────────────────────────────────────┘')
        
        return True
        
    def calculate_displacement(self):
        """Calculer le déplacement depuis le début"""
        if self.initial_odom is None or self.last_odom is None:
            return None
            
        initial_pos = self.get_position(self.initial_odom)
        current_pos = self.get_position(self.last_odom)
        
        dx = current_pos[0] - initial_pos[0]
        dy = current_pos[1] - initial_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        initial_yaw = self.get_yaw(self.initial_odom)
        current_yaw = self.get_yaw(self.last_odom)
        angle_change = math.degrees(current_yaw - initial_yaw)
        
        return {
            'distance': distance,
            'angle': angle_change,
            'dx': dx,
            'dy': dy
        }
    
    def run_test(self):
        """Exécuter la séquence de test"""
        self.get_logger().info('\n📡 Phase 1: Vérification du topic /odom...')
        time.sleep(2)
        
        if not self.odom_received:
            self.get_logger().error('❌ ÉCHEC: Topic /odom non reçu !')
            self.get_logger().error('   Vérifiez que Gazebo et les contrôleurs sont lancés')
            return False
        
        self.get_logger().info('\n📊 État initial de l\'odométrie:')
        self.display_odom_info()
        
        # Test 1: Mouvement avant
        self.get_logger().info('\n🧪 TEST 1: Mouvement en avant')
        self.move_forward(2.0)
        time.sleep(1)
        
        self.get_logger().info('\n📊 Odométrie après mouvement avant:')
        self.display_odom_info()
        
        disp = self.calculate_displacement()
        if disp:
            self.get_logger().info(f'\n📏 Déplacement mesuré:')
            self.get_logger().info(f'   Distance: {disp["distance"]:.3f} m')
            self.get_logger().info(f'   Δx: {disp["dx"]:.3f} m, Δy: {disp["dy"]:.3f} m')
            
            if disp['distance'] > 0.05:
                self.get_logger().info('✅ L\'odométrie détecte le mouvement avant !')
            else:
                self.get_logger().warn('⚠️  Mouvement très faible ou nul détecté')
        
        # Test 2: Rotation
        self.get_logger().info('\n🧪 TEST 2: Rotation sur place')
        self.rotate(2.0)
        time.sleep(1)
        
        self.get_logger().info('\n📊 Odométrie après rotation:')
        self.display_odom_info()
        
        disp = self.calculate_displacement()
        if disp:
            self.get_logger().info(f'\n🔄 Rotation mesurée: {disp["angle"]:.1f}°')
            
            if abs(disp['angle']) > 5:
                self.get_logger().info('✅ L\'odométrie détecte la rotation !')
            else:
                self.get_logger().warn('⚠️  Rotation très faible ou nulle détectée')
        
        # Résumé
        self.get_logger().info('\n╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║               RÉSUMÉ DU TEST                             ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
        if self.odom_received and disp and disp['distance'] > 0.05:
            self.get_logger().info('✅ SUCCÈS: L\'odométrie fonctionne correctement !')
            self.get_logger().info('   - Topic /odom reçu')
            self.get_logger().info('   - Mouvement avant détecté')
            self.get_logger().info('   - Rotation détectée')
            self.get_logger().info('\n💡 L\'odométrie utilise les encodeurs des roues')
            self.get_logger().info('   Elle ne dépend PAS du LIDAR')
            return True
        else:
            self.get_logger().warn('⚠️  PROBLÈME: L\'odométrie ne fonctionne pas correctement')
            self.get_logger().warn('   Vérifiez les contrôleurs diff_drive')
            return False

def main(args=None):
    rclpy.init(args=args)
    tester = OdometryTester()
    
    try:
        tester.run_test()
        time.sleep(2)
    except KeyboardInterrupt:
        tester.get_logger().info('⚠️  Test interrompu')
    finally:
        tester.stop()
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
