#!/usr/bin/env python3
"""
Système Autonome INTELLIGENT de Pick and Store - Version 2
===========================================================
Explore l'environnement avec LIDAR, détecte les packages,
les collecte et les stocke automatiquement.

Usage:
    ros2 run my_robot_controller autonomous_explorer.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import time


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/detected_objects', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # État du robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0
        
        # Objets détectés
        self.packages = []          # Packages à collecter
        self.storage_zone = None    # Zone de stockage
        
        # Configuration détection
        self.PACKAGE_MIN_SIZE = 0.08   # Taille min (réduite pour petits packages)
        self.PACKAGE_MAX_SIZE = 1.2    # Taille max 
        self.DETECTION_RANGE = 5.0     # Distance de détection augmentée
        
        # Bras - Optimisé pour PETITS packages
        self.ARM_HOME = [-0.5, 2.5, 0.0]
        self.ARM_REACH = [1.0, 0.5, 0.0]  # Plus bas et plus étendu pour petits objets
        self.ARM_LIFT = [0.0, 1.5, 0.0]
        self.ARM_PLACE = [0.4, 1.0, 0.0]
        self.GRIPPER_OPEN = -0.12  # Plus ouvert
        self.GRIPPER_CLOSED = 0.0
        
        # Vitesses
        self.linear_speed = 0.15
        self.angular_speed = 0.3
        
        self.packages_collected = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 EXPLORATEUR AUTONOME V2')
        self.get_logger().info('=' * 60)
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg):
        self.scan_ranges = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment
    
    def stop(self):
        self.cmd_vel_pub.publish(Twist())
    
    def wait_for_data(self, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan_ranges:
                return True
        return False
    
    def get_distance_to(self, x, y):
        return math.sqrt((x - self.current_x)**2 + (y - self.current_y)**2)
    
    def get_angle_to(self, x, y):
        return math.atan2(y - self.current_y, x - self.current_x)
    
    # ==================== DÉTECTION LIDAR ====================
    
    def detect_objects(self):
        """Détecte les objets depuis les données LIDAR"""
        if not self.scan_ranges:
            return []
        
        objects = []
        current_points = []
        prev_dist = float('inf')
        
        num_rays = len(self.scan_ranges)
        
        for i in range(num_rays):
            dist = self.scan_ranges[i]
            
            # Ignorer invalides
            if dist < 0.15 or dist > self.DETECTION_RANGE or math.isinf(dist) or math.isnan(dist):
                if len(current_points) >= 3:
                    obj = self._analyze_points(current_points)
                    if obj:
                        objects.append(obj)
                current_points = []
                prev_dist = float('inf')
                continue
            
            # Position en coordonnées monde
            angle = self.scan_angle_min + i * self.scan_angle_increment
            world_angle = self.current_yaw + angle
            px = self.current_x + dist * math.cos(world_angle)
            py = self.current_y + dist * math.sin(world_angle)
            
            # Discontinuité = nouvel objet
            if abs(dist - prev_dist) > 0.4:
                if len(current_points) >= 3:
                    obj = self._analyze_points(current_points)
                    if obj:
                        objects.append(obj)
                current_points = []
            
            current_points.append({'x': px, 'y': py, 'dist': dist})
            prev_dist = dist
        
        # Dernier groupe
        if len(current_points) >= 3:
            obj = self._analyze_points(current_points)
            if obj:
                objects.append(obj)
        
        return objects
    
    def _analyze_points(self, points):
        """Analyse un groupe de points"""
        xs = [p['x'] for p in points]
        ys = [p['y'] for p in points]
        
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        width = max_x - min_x
        height = max_y - min_y
        size = math.sqrt(width**2 + height**2)
        
        return {
            'x': (min_x + max_x) / 2,
            'y': (min_y + max_y) / 2,
            'size': size,
            'num_points': len(points)
        }
    
    def classify_objects(self, objects):
        """Classifie en packages et étagère"""
        packages = []
        shelf_candidates = []
        
        for obj in objects:
            size = obj['size']
            x, y = obj['x'], obj['y']
            num_pts = obj['num_points']
            
            self.get_logger().info(
                f'   Objet: ({x:.2f}, {y:.2f}), taille={size:.2f}m, pts={num_pts}'
            )
            
            # Package: petite/moyenne taille
            # - Taille entre 0.08 et 1.2m
            # - Moins de 80 points (pas un mur)
            # - Dans la zone de travail
            if self.PACKAGE_MIN_SIZE <= size <= self.PACKAGE_MAX_SIZE:
                if num_pts < 80:
                    if -7 < x < 7 and -7 < y < 7:
                        # Vérifier que c'est pas trop près des murs (bords)
                        if abs(x) < 7.5 and abs(y) < 7.5:
                            packages.append({'x': x, 'y': y, 'size': size, 'picked': False})
                            self.get_logger().info(f'   → 📦 PACKAGE DÉTECTÉ!')
                        else:
                            self.get_logger().info(f'   → ❌ Trop près du mur')
                    else:
                        self.get_logger().info(f'   → ❌ Hors zone')
                else:
                    self.get_logger().info(f'   → ❌ Trop de points (mur?)')
            
            # Étagère: grande structure
            elif size > 1.5:
                shelf_candidates.append(obj)
                self.get_logger().info(f'   → 📍 Structure (étagère potentielle)')
        
        # Zone de stockage - prendre la plus grande structure
        storage = None
        if shelf_candidates:
            best = max(shelf_candidates, key=lambda x: x['size'])
            # Zone de dépôt = devant l'étagère
            storage = {'x': best['x'] - 1.5, 'y': best['y']}
            self.get_logger().info(f'   📍 Zone stockage: ({storage["x"]:.2f}, {storage["y"]:.2f})')
        
        return packages, storage
    
    # ==================== EXPLORATION ====================
    
    def rotate_and_scan(self):
        """Tourne 360° et détecte les objets"""
        self.get_logger().info('🔄 Rotation et scan...')
        
        # Rotation complète
        twist = Twist()
        twist.angular.z = self.angular_speed
        
        start_time = time.time()
        rotation_time = 2 * math.pi / self.angular_speed  # Temps pour 360°
        
        while time.time() - start_time < rotation_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop()
        time.sleep(0.3)
        
        # Analyser
        rclpy.spin_once(self, timeout_sec=0.3)
        objects = self.detect_objects()
        packages, storage = self.classify_objects(objects)
        
        return packages, storage
    
    def navigate_to(self, target_x, target_y, tolerance=0.5):
        """Navigation simple"""
        self.get_logger().info(f'🚗 Navigation vers ({target_x:.1f}, {target_y:.1f})')
        
        for _ in range(400):
            dist = self.get_distance_to(target_x, target_y)
            
            if dist < tolerance:
                self.stop()
                self.get_logger().info('✅ Arrivé')
                return True
            
            angle_to = self.get_angle_to(target_x, target_y)
            angle_diff = angle_to - self.current_yaw
            
            # Normaliser
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            
            twist = Twist()
            
            # Évitement obstacles
            if self._obstacle_ahead():
                twist.angular.z = 0.5
                twist.linear.x = 0.0
            elif abs(angle_diff) > 0.3:
                twist.angular.z = 0.4 if angle_diff > 0 else -0.4
                twist.linear.x = 0.02
            else:
                twist.linear.x = min(0.2, dist * 0.5)
                twist.angular.z = angle_diff * 0.5
            
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.stop()
        return False
    
    def _obstacle_ahead(self, threshold=0.35):
        """Vérifie obstacle devant"""
        if not self.scan_ranges:
            return False
        
        n = len(self.scan_ranges)
        front = n // 2
        margin = n // 10
        
        for i in range(front - margin, front + margin):
            if 0 <= i < n:
                r = self.scan_ranges[i]
                if 0.1 < r < threshold:
                    return True
        return False
    
    def full_exploration(self):
        """Exploration complète"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🗺️  EXPLORATION')
        self.get_logger().info('='*60)
        
        all_packages = []
        storage = None
        
        # Points d'exploration - PROCHES des positions réelles des packages
        # Packages sont à: (-3,5), (-2,5.5), (0,5.5), (2,5)
        # Étagère à: (6,-4)
        points = [
            (0, 0),       # Centre - scan initial
            (0, 3),       # Approche zone packages
            (-2, 3),      # Près des petits packages
            (1, 3),       # Près des grands packages
            (3, 0),       # Vers l'étagère
            (4, -2),      # Devant l'étagère
        ]
        
        for i, (ex, ey) in enumerate(points):
            self.get_logger().info(f'\n📍 Point {i+1}/{len(points)}: ({ex}, {ey})')
            
            # Naviguer (même si échec partiel, on scanne quand même)
            self.navigate_to(ex, ey)
            
            # TOUJOURS scanner à chaque point
            pkgs, found_storage = self.rotate_and_scan()
            
            # Ajouter nouveaux packages
            for pkg in pkgs:
                already_exists = False
                for existing in all_packages:
                    dist = math.sqrt((pkg['x']-existing['x'])**2 + (pkg['y']-existing['y'])**2)
                    if dist < 0.6:
                        already_exists = True
                        break
                if not already_exists:
                    all_packages.append(pkg)
                    self.get_logger().info(f'📦 Nouveau: ({pkg["x"]:.2f}, {pkg["y"]:.2f})')
            
            if found_storage:
                storage = found_storage
            
            # Continuer même si on a des packages (explorer tout)
            if len(all_packages) >= 4 and storage:
                break
        
        # Résumé
        self.get_logger().info('\n' + '-'*50)
        self.get_logger().info(f'📊 Packages: {len(all_packages)}')
        for p in all_packages:
            self.get_logger().info(f'   ({p["x"]:.2f}, {p["y"]:.2f})')
        
        if not storage:
            self.get_logger().warn('⚠️ Stockage par défaut')
            storage = {'x': 4.0, 'y': -4.0}
        self.get_logger().info(f'📍 Stockage: ({storage["x"]:.2f}, {storage["y"]:.2f})')
        
        self.packages = all_packages
        self.storage_zone = storage
        return all_packages, storage
    
    # ==================== CONTRÔLE BRAS ====================
    
    def move_arm(self, positions, duration=2.0):
        self.get_logger().info(f'🦾 Bras → {positions}')
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        traj.points = [point]
        self.arm_pub.publish(traj)
        time.sleep(duration + 0.3)
    
    def control_gripper(self, position, duration=1.0):
        state = "OUVERTE" if position < 0 else "FERMÉE"
        self.get_logger().info(f'✋ Pince {state}')
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        traj.points = [point]
        self.gripper_pub.publish(traj)
        time.sleep(duration + 0.2)
    
    # ==================== PICK & PLACE ====================
    
    def rotate_to_angle(self, target_angle):
        for _ in range(150):
            diff = target_angle - self.current_yaw
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            
            if abs(diff) < 0.1:
                break
            
            twist = Twist()
            twist.angular.z = 0.3 if diff > 0 else -0.3
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()
    
    def approach_slowly(self, target_x, target_y, stop_dist=0.4):
        angle = self.get_angle_to(target_x, target_y)
        self.rotate_to_angle(angle)
        
        for _ in range(150):
            if self.get_distance_to(target_x, target_y) < stop_dist:
                self.stop()
                return True
            twist = Twist()
            twist.linear.x = 0.08
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()
        return True
    
    def backup(self, distance=0.5):
        twist = Twist()
        twist.linear.x = -0.1
        for _ in range(int(distance * 200)):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()
    
    def pick_package(self, package):
        pkg_x = package["x"]
        pkg_y = package["y"]
        pkg_size = package.get("size", 0.3)
        
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'📦 PICK PACKAGE')
        self.get_logger().info(f'   Position: ({pkg_x:.2f}, {pkg_y:.2f})')
        self.get_logger().info(f'   Taille: {pkg_size:.2f}m')
        self.get_logger().info(f'{"="*50}')
        
        # 1. Replier le bras pour navigation
        self.get_logger().info('Phase 1: Préparation bras')
        self.move_arm(self.ARM_HOME, 1.5)
        
        # 2. Naviguer PLUS PRÈS du package (0.6m seulement)
        self.get_logger().info('Phase 2: Navigation vers package')
        self.navigate_to(pkg_x, pkg_y, tolerance=0.6)
        
        # 3. Ouvrir la pince GRAND
        self.get_logger().info('Phase 3: Ouverture pince')
        self.control_gripper(self.GRIPPER_OPEN)
        time.sleep(0.5)
        
        # 4. Étendre le bras vers le package
        self.get_logger().info('Phase 4: Extension du bras')
        self.move_arm(self.ARM_REACH, 2.5)
        
        # 5. Approche finale TRÈS PROCHE (0.25m pour petits packages)
        self.get_logger().info('Phase 5: Approche finale')
        self.approach_slowly(pkg_x, pkg_y, stop_dist=0.25)
        time.sleep(0.8)
        
        # 6. Fermer la pince DOUCEMENT (saisir)
        self.get_logger().info('Phase 6: Fermeture pince - SAISIE')
        self.control_gripper(self.GRIPPER_CLOSED, duration=1.5)  # Plus lent pour meilleure prise
        time.sleep(1.5)  # Attendre que la pince se ferme bien
        
        # 7. Lever le bras LENTEMENT avec le package
        self.get_logger().info('Phase 7: Levée du bras')
        self.move_arm(self.ARM_LIFT, 3.0)  # Plus lent pour stabilité
        time.sleep(1.0)
        
        # 8. Reculer pour se dégager
        self.get_logger().info('Phase 8: Recul')
        self.backup(0.5)
        
        package['picked'] = True
        self.get_logger().info('✅ Package saisi avec succès!')
        return True
    
    def place_package(self, storage_x, storage_y):
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'📥 PLACE PACKAGE')
        self.get_logger().info(f'   Destination: ({storage_x:.2f}, {storage_y:.2f})')
        self.get_logger().info(f'{"="*50}')
        
        # 1. Garder le bras levé pendant le transport
        self.get_logger().info('Phase 1: Bras en position transport')
        self.move_arm(self.ARM_LIFT, 1.0)
        
        # 2. Naviguer vers la zone de stockage
        self.get_logger().info('Phase 2: Navigation vers stockage')
        self.navigate_to(storage_x, storage_y, tolerance=0.6)
        
        # 3. Approche finale
        self.get_logger().info('Phase 3: Approche finale')
        self.approach_slowly(storage_x, storage_y, stop_dist=0.5)
        time.sleep(0.3)
        
        # 4. Descendre le bras
        self.get_logger().info('Phase 4: Descente du bras')
        self.move_arm(self.ARM_PLACE, 2.0)
        time.sleep(0.5)
        
        # 5. Ouvrir la pince (libérer)
        self.get_logger().info('Phase 5: Ouverture pince - DÉPÔT')
        self.control_gripper(self.GRIPPER_OPEN)
        time.sleep(1.0)
        
        # 6. Replier le bras
        self.get_logger().info('Phase 6: Repli du bras')
        self.move_arm(self.ARM_HOME, 1.5)
        
        # 7. Reculer
        self.get_logger().info('Phase 7: Recul')
        self.backup(0.5)
        
        self.packages_collected += 1
        self.get_logger().info('✅ Package déposé avec succès!')
        return True
    
    # ==================== MISSION ====================
    
    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, pkg in enumerate(self.packages):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "packages"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = pkg['x']
            marker.pose.position.y = pkg['y']
            marker.pose.position.z = 0.25
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5) if pkg.get('picked') else ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            marker_array.markers.append(marker)
        
        if self.storage_zone:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "storage"
            marker.id = 100
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = self.storage_zone['x']
            marker.pose.position.y = self.storage_zone['y']
            marker.pose.position.z = 0.1
            marker.scale.x = marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def run_mission(self):
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🚀 MISSION AUTONOME - UN PACKAGE À LA FOIS')
        self.get_logger().info('='*60)
        
        # Attendre données
        self.get_logger().info('⏳ Attente données LIDAR...')
        if not self.wait_for_data(10.0):
            self.get_logger().error('❌ Pas de données!')
            return 0
        
        self.get_logger().info(f'📍 Position: ({self.current_x:.2f}, {self.current_y:.2f})')
        
        # Init bras
        self.move_arm(self.ARM_HOME, 2.0)
        self.control_gripper(self.GRIPPER_OPEN)
        
        # Exploration
        packages, storage = self.full_exploration()
        
        if not packages:
            self.get_logger().warn('⚠️ Aucun package!')
            return 0
        
        # TRIER par taille (plus petit d'abord = packages bleus)
        packages.sort(key=lambda p: p['size'])
        
        self.get_logger().info('\n📦 Packages triés par taille (petit → grand):')
        for i, p in enumerate(packages):
            self.get_logger().info(f'   {i+1}. ({p["x"]:.2f}, {p["y"]:.2f}) - taille: {p["size"]:.2f}m')
        
        self.publish_markers()
        
        # PRENDRE SEULEMENT LE PREMIER (le plus petit)
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('📦 COLLECTE DU PREMIER PACKAGE (le plus petit)')
        self.get_logger().info('='*60)
        
        pkg = packages[0]  # Premier package (le plus petit)
        self.get_logger().info(f'\n🎯 Cible: ({pkg["x"]:.2f}, {pkg["y"]:.2f}) - taille: {pkg["size"]:.2f}m')
        
        if self.pick_package(pkg):
            store_y = storage['y']
            self.place_package(storage['x'], store_y)
        
        self.publish_markers()
        
        # Retour
        self.get_logger().info('\n🏠 Retour...')
        self.navigate_to(0, 0)
        self.move_arm(self.ARM_HOME)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🎉 MISSION TERMINÉE!')
        self.get_logger().info(f'📦 Collectés: {self.packages_collected}/1')
        self.get_logger().info('='*60)
        
        return self.packages_collected


def main(args=None):
    rclpy.init(args=args)
    robot = AutonomousExplorer()
    
    try:
        robot.get_logger().info('⏳ Attente 5 secondes...')
        time.sleep(5.0)
        robot.run_mission()
    except KeyboardInterrupt:
        robot.get_logger().info('\n⚠️ Interruption')
        robot.stop()
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
