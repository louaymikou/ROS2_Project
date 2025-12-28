#!/usr/bin/env python3
"""
Système Autonome de Pick and Store
===================================
Ce script fait tout automatiquement :
1. Détecte les packages dans l'environnement (positions connues)
2. Navigue vers chaque package
3. Prend le package avec la pince
4. Le transporte vers la zone de stockage
5. Le dépose sur l'étagère

Pour lancer:
    ros2 run my_robot_controller autonomous_pick_and_store.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import math
import time
import asyncio


class AutonomousPickAndStore(Node):
    """
    Robot autonome qui détecte, collecte et stocke les packages
    """
    
    def __init__(self):
        super().__init__('autonomous_pick_and_store')
        
        callback_group = ReentrantCallbackGroup()
        
        # Publishers pour contrôle manuel de backup
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Subscriber pour position
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_cont/odom', self.odom_callback, 10,
            callback_group=callback_group)
        
        # Subscriber LIDAR pour détection d'obstacles
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=callback_group)
        
        # Nav2 Action Client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=callback_group)
        
        # État du robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.scan_data = None
        
        # ========== CONFIGURATION DES PACKAGES ==========
        # Positions des packages dans votre monde (depuis my_world.world)
        self.packages = [
            {'name': 'small_package_1', 'x': -3.0, 'y': 5.0, 'size': 'small', 'picked': False},
            {'name': 'small_package_2', 'x': -2.0, 'y': 5.5, 'size': 'small', 'picked': False},
            {'name': 'large_package_1', 'x': 0.0, 'y': 5.5, 'size': 'large', 'picked': False},
            {'name': 'large_package_2', 'x': 2.0, 'y': 5.0, 'size': 'large', 'picked': False},
        ]
        
        # Zone de stockage (devant l'étagère)
        self.storage_locations = [
            {'x': 4.5, 'y': -5.0, 'used': False},  # Casier gauche
            {'x': 4.5, 'y': -4.0, 'used': False},  # Casier centre
            {'x': 4.5, 'y': -3.0, 'used': False},  # Casier droite
            {'x': 4.5, 'y': -5.5, 'used': False},  # Extra
        ]
        
        # Positions du bras
        self.ARM_HOME = [-0.5, 2.5, 0.0]      # Bras replié pour navigation
        self.ARM_REACH = [0.6, 0.8, 0.0]       # Bras étendu pour saisir
        self.ARM_LIFT = [0.0, 1.5, 0.0]        # Bras levé avec objet
        self.ARM_PLACE = [0.4, 1.0, 0.0]       # Bras pour poser
        
        self.GRIPPER_OPEN = -0.10   # Pince ouverte
        self.GRIPPER_CLOSED = 0.0   # Pince fermée
        
        # Vitesses
        self.linear_speed = 0.15
        self.angular_speed = 0.3
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 SYSTÈME AUTONOME PICK & STORE INITIALISÉ')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📦 Packages à collecter: {len(self.packages)}')
        self.get_logger().info(f'📍 Zones de stockage: {len(self.storage_locations)}')
        
    def odom_callback(self, msg):
        """Met à jour la position du robot"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extraire l'angle yaw du quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg):
        """Stocke les données LIDAR"""
        self.scan_data = msg
    
    def get_distance_to(self, target_x, target_y):
        """Calcule la distance vers une cible"""
        return math.sqrt((target_x - self.current_x)**2 + 
                        (target_y - self.current_y)**2)
    
    def get_angle_to(self, target_x, target_y):
        """Calcule l'angle vers une cible"""
        return math.atan2(target_y - self.current_y, 
                         target_x - self.current_x)
    
    # ==================== CONTRÔLE DU BRAS ====================
    
    def move_arm(self, positions, duration=2.0):
        """Bouge le bras vers les positions spécifiées"""
        self.get_logger().info(f'🦾 Bras → {positions}')
        
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        traj.points = [point]
        self.arm_pub.publish(traj)
        time.sleep(duration + 0.5)
    
    def control_gripper(self, position, duration=1.0):
        """Ouvre ou ferme la pince"""
        state = "🔓 OUVERTE" if position < 0 else "🔒 FERMÉE"
        self.get_logger().info(f'✋ Pince {state}')
        
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        traj.points = [point]
        self.gripper_pub.publish(traj)
        time.sleep(duration + 0.3)
    
    # ==================== NAVIGATION ====================
    
    def stop(self):
        """Arrête le robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def move_forward(self, distance, speed=None):
        """Avance d'une certaine distance"""
        if speed is None:
            speed = self.linear_speed
            
        start_x, start_y = self.current_x, self.current_y
        
        twist = Twist()
        twist.linear.x = speed
        
        while self.get_distance_to(start_x, start_y) < distance:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop()
    
    def rotate_to_angle(self, target_angle, tolerance=0.05):
        """Tourne vers un angle cible (en radians)"""
        twist = Twist()
        
        while True:
            angle_diff = target_angle - self.current_yaw
            # Normaliser l'angle entre -pi et pi
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if abs(angle_diff) < tolerance:
                break
            
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop()
    
    def navigate_to(self, target_x, target_y, approach_distance=0.5):
        """
        Navigation simple vers une cible
        Approche à une certaine distance de la cible
        """
        self.get_logger().info(f'🚗 Navigation vers ({target_x:.1f}, {target_y:.1f})')
        
        # Calcul du point d'approche (devant la cible)
        angle_to_target = self.get_angle_to(target_x, target_y)
        approach_x = target_x - approach_distance * math.cos(angle_to_target)
        approach_y = target_y - approach_distance * math.sin(angle_to_target)
        
        iteration = 0
        max_iterations = 200
        
        while iteration < max_iterations:
            distance = self.get_distance_to(approach_x, approach_y)
            
            if distance < 0.15:  # Arrivé
                self.get_logger().info('✅ Position atteinte!')
                self.stop()
                return True
            
            # Calcul de l'angle vers la cible
            angle_to_target = self.get_angle_to(approach_x, approach_y)
            angle_diff = angle_to_target - self.current_yaw
            
            # Normaliser
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            twist = Twist()
            
            # Si mal orienté, tourner d'abord
            if abs(angle_diff) > 0.3:
                twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                twist.linear.x = 0.05  # Avance lente
            else:
                # Bien orienté, avancer
                twist.linear.x = min(self.linear_speed, distance * 0.5)
                twist.angular.z = angle_diff * 0.8  # Correction légère
            
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            iteration += 1
        
        self.get_logger().warn('⚠️ Navigation timeout')
        self.stop()
        return False
    
    def approach_target(self, target_x, target_y, final_distance=0.35):
        """Approche finale très lente vers la cible"""
        self.get_logger().info(f'🎯 Approche finale vers ({target_x:.1f}, {target_y:.1f})')
        
        # D'abord tourner vers la cible
        angle_to_target = self.get_angle_to(target_x, target_y)
        self.rotate_to_angle(angle_to_target)
        time.sleep(0.3)
        
        # Avancer lentement
        iteration = 0
        while iteration < 100:
            distance = self.get_distance_to(target_x, target_y)
            
            if distance < final_distance:
                self.stop()
                return True
            
            twist = Twist()
            twist.linear.x = 0.08  # Très lent
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            iteration += 1
        
        self.stop()
        return True
    
    # ==================== SÉQUENCES PICK & PLACE ====================
    
    def pick_package(self, package):
        """
        Séquence complète pour prendre un package
        """
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'📦 PICK: {package["name"]}')
        self.get_logger().info(f'📍 Position: ({package["x"]}, {package["y"]})')
        self.get_logger().info(f'{"="*50}')
        
        # 1. Replier le bras pour la navigation
        self.get_logger().info('Phase 1: Préparation du bras')
        self.move_arm(self.ARM_HOME, 1.5)
        
        # 2. Naviguer vers le package
        self.get_logger().info('Phase 2: Navigation vers le package')
        if not self.navigate_to(package['x'], package['y'], approach_distance=0.6):
            self.get_logger().error('❌ Échec navigation')
            return False
        
        # 3. Ouvrir la pince
        self.get_logger().info('Phase 3: Ouverture de la pince')
        self.control_gripper(self.GRIPPER_OPEN)
        
        # 4. Étendre le bras
        self.get_logger().info('Phase 4: Extension du bras')
        self.move_arm(self.ARM_REACH, 2.0)
        
        # 5. Approche finale
        self.get_logger().info('Phase 5: Approche finale')
        self.approach_target(package['x'], package['y'], final_distance=0.30)
        time.sleep(0.5)
        
        # 6. Fermer la pince
        self.get_logger().info('Phase 6: Saisie du package')
        self.control_gripper(self.GRIPPER_CLOSED)
        time.sleep(1.0)
        
        # 7. Lever le bras
        self.get_logger().info('Phase 7: Levée du bras')
        self.move_arm(self.ARM_LIFT, 2.0)
        
        # 8. Reculer un peu
        self.get_logger().info('Phase 8: Recul')
        twist = Twist()
        twist.linear.x = -0.1
        for _ in range(30):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()
        
        self.get_logger().info('✅ Package saisi avec succès!')
        return True
    
    def place_package(self, storage):
        """
        Séquence complète pour déposer un package
        """
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'📥 PLACE au stockage')
        self.get_logger().info(f'📍 Position: ({storage["x"]}, {storage["y"]})')
        self.get_logger().info(f'{"="*50}')
        
        # 1. Garder le bras levé pendant navigation
        self.get_logger().info('Phase 1: Bras en position de transport')
        self.move_arm(self.ARM_LIFT, 1.0)
        
        # 2. Naviguer vers la zone de stockage
        self.get_logger().info('Phase 2: Navigation vers stockage')
        if not self.navigate_to(storage['x'], storage['y'], approach_distance=0.5):
            self.get_logger().error('❌ Échec navigation stockage')
            return False
        
        # 3. Approche finale
        self.get_logger().info('Phase 3: Approche finale')
        self.approach_target(storage['x'], storage['y'], final_distance=0.40)
        
        # 4. Baisser le bras
        self.get_logger().info('Phase 4: Descente du bras')
        self.move_arm(self.ARM_PLACE, 2.0)
        time.sleep(0.5)
        
        # 5. Ouvrir la pince
        self.get_logger().info('Phase 5: Libération du package')
        self.control_gripper(self.GRIPPER_OPEN)
        time.sleep(1.0)
        
        # 6. Replier le bras
        self.get_logger().info('Phase 6: Repli du bras')
        self.move_arm(self.ARM_HOME, 1.5)
        
        # 7. Reculer
        self.get_logger().info('Phase 7: Recul')
        twist = Twist()
        twist.linear.x = -0.15
        for _ in range(40):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()
        
        self.get_logger().info('✅ Package déposé avec succès!')
        return True
    
    # ==================== MISSION PRINCIPALE ====================
    
    def run_mission(self):
        """
        Mission complète:
        - Collecter tous les packages
        - Les stocker dans la zone de stockage
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🚀 DÉMARRAGE DE LA MISSION AUTONOME')
        self.get_logger().info('='*60)
        
        # Attendre que l'odométrie soit prête
        self.get_logger().info('⏳ Attente initialisation odométrie...')
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().info(f'📍 Position initiale: ({self.current_x:.2f}, {self.current_y:.2f})')
        
        # Préparer le robot
        self.get_logger().info('\n🔧 Initialisation du robot...')
        self.move_arm(self.ARM_HOME, 2.0)
        self.control_gripper(self.GRIPPER_OPEN)
        
        packages_collected = 0
        storage_idx = 0
        
        # Collecter chaque package
        for i, package in enumerate(self.packages):
            if package['picked']:
                continue
            
            self.get_logger().info(f'\n{"#"*60}')
            self.get_logger().info(f'📦 PACKAGE {i+1}/{len(self.packages)}: {package["name"]}')
            self.get_logger().info(f'{"#"*60}')
            
            # Prendre le package
            if self.pick_package(package):
                package['picked'] = True
                
                # Trouver un emplacement de stockage libre
                while storage_idx < len(self.storage_locations):
                    storage = self.storage_locations[storage_idx]
                    if not storage['used']:
                        break
                    storage_idx += 1
                
                if storage_idx >= len(self.storage_locations):
                    self.get_logger().warn('⚠️ Plus de place de stockage!')
                    break
                
                # Déposer le package
                if self.place_package(storage):
                    storage['used'] = True
                    packages_collected += 1
                    storage_idx += 1
                    self.get_logger().info(f'✅ Package {i+1} stocké!')
                else:
                    self.get_logger().error(f'❌ Échec dépôt package {i+1}')
            else:
                self.get_logger().error(f'❌ Échec collecte package {i+1}')
            
            # Petite pause entre les packages
            time.sleep(1.0)
        
        # Retour à la position initiale
        self.get_logger().info('\n🏠 Retour à la position initiale...')
        self.navigate_to(0.0, 0.0, approach_distance=0.3)
        self.move_arm(self.ARM_HOME)
        
        # Résumé
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🎉 MISSION TERMINÉE!')
        self.get_logger().info(f'📦 Packages collectés: {packages_collected}/{len(self.packages)}')
        self.get_logger().info('='*60)
        
        return packages_collected


def main(args=None):
    rclpy.init(args=args)
    
    robot = AutonomousPickAndStore()
    
    try:
        # Laisser le temps au robot de s'initialiser
        robot.get_logger().info('⏳ Attente 5 secondes pour initialisation...')
        time.sleep(5.0)
        
        # Lancer la mission
        collected = robot.run_mission()
        
        robot.get_logger().info(f'\n✅ Mission terminée. {collected} packages stockés.')
        
    except KeyboardInterrupt:
        robot.get_logger().info('\n⚠️ Mission interrompue par utilisateur')
        robot.stop()
        robot.move_arm(robot.ARM_HOME)
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
