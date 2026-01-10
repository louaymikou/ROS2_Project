#!/usr/bin/env python3
"""
ArUco Navigation Action Server
================================
Ce serveur d'action permet de naviguer jusqu'à un marqueur ArUco spécifique.
- Si le numéro du marqueur cible est avant (inférieur) la position actuelle: marche arrière
- Si le numéro du marqueur cible est après (supérieur) la position actuelle: marche avant
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from custom_interfaces.action import NavigateToAruco
from sensor_msgs.msg import Image, Range
from std_srvs.srv import SetBool
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from enum import Enum


class NavigationState(Enum):
    """États de la machine à états de navigation"""
    IDLE = 0
    NAVIGATE_TO_TARGET = 1          # Suivi ligne bleue jusqu'à la cible
    ROTATE_AT_TARGET = 2             # Rotation gauche/droite à la cible
    FOLLOW_COLOR_TO_OBSTACLE = 3     # Suivi ligne rouge/verte jusqu'à obstacle
    OBSTACLE_DETECTED = 4            # Obstacle détecté, robot arrêté
    RETURN_ON_COLOR = 5              # Retour sur ligne rouge/verte
    ROTATE_AT_MARKER = 6             # Rotation pour retrouver ligne bleue
    RETURN_TO_BASE = 7               # Retour sur ligne bleue jusqu'à ArUco 0


class ArucoNavigationServer(Node):
    """
    Serveur d'action pour la navigation vers des marqueurs ArUco.
    """

    def __init__(self):
        super().__init__('aruco_navigation_server')
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Track current ArUco ID detected
        self.current_aruco_id = None
        self.aruco_detection_count = 0
        self.required_detections = 1  # Nombre de détections consécutives requises (1 pour plus de réactivité)
        
        # ArUco position tracking for centering
        self.aruco_center_x = None
        self.aruco_center_y = None
        self.image_center_x = None
        self.image_center_y = None
        
        # Obstacle detection
        self.front_obstacle_distance = float('inf')
        self.rear_obstacle_distance = float('inf')
        self.obstacle_threshold = 0.3  # 30cm
        
        # Line detection tracking
        self.line_detected = True
        self.last_line_detection_time = time.time()
        
        # Subscribe to bottom camera (fixed under chassis for ArUco detection only)
        self.bottom_subscription = self.create_subscription(
            Image,
            '/bottom_camera/image_raw',
            self.bottom_camera_callback,
            10)
        
        # Subscribe to ultrasonic sensors
        self.front_ultrasonic_subscription = self.create_subscription(
            Range,
            '/front_ultrasonic/range',
            self.front_ultrasonic_callback,
            10)
        
        self.rear_ultrasonic_subscription = self.create_subscription(
            Range,
            '/rear_ultrasonic/range',
            self.rear_ultrasonic_callback,
            10)
        
        # Subscribe to line detection status
        from std_msgs.msg import Bool
        self.line_detection_subscription = self.create_subscription(
            Bool,
            '/line_detected',
            self.line_detection_callback,
            10)
        
        # Publisher pour afficher les images avec détections ArUco (bottom camera uniquement)
        self.bottom_aruco_pub = self.create_publisher(
            Image,
            '/bottom_camera/aruco_detection',
            10)
        
        # Service clients pour contrôler le robot
        self.enable_movement_client = self.create_client(SetBool, 'enable_movement')
        self.set_direction_client = self.create_client(SetBool, 'set_forward_direction')
        
        # Publisher pour changer la couleur de ligne suivie
        self.color_change_publisher = self.create_publisher(String, '/set_line_color', 10)
        
        # Action server
        self.action_server = ActionServer(
            self,
            NavigateToAruco,
            'navigate_to_aruco',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # État de navigation
        self.is_navigating = False
        self.target_id = None
        self.start_time = None
        self.went_forward = True
        self.current_state = NavigationState.IDLE
        self.rotation_direction = None  # 'left' or 'right'
        
        # Publisher pour contrôler directement le robot si nécessaire
        self.cmd_vel_publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # Vitesses pour les rotations
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        
        self.get_logger().info('=================================')
        self.get_logger().info('🎯 ArUco Navigation Server Ready')
        self.get_logger().info('   Action: /navigate_to_aruco')
        self.get_logger().info(f'   Obstacle detection: {self.obstacle_threshold}m')
        self.get_logger().info('   ArUco detection:')
        self.get_logger().info('     - /bottom_camera/aruco_detection (bottom camera only)')
        self.get_logger().info('   Line following cameras:')
        self.get_logger().info('     - /camera/image_raw (front)')
        self.get_logger().info('     - /rear_camera/image_raw (rear)')
        self.get_logger().info('=================================')

    def front_ultrasonic_callback(self, msg):
        """Callback pour le capteur ultrason avant"""
        self.front_obstacle_distance = msg.range

    def rear_ultrasonic_callback(self, msg):
        """Callback pour le capteur ultrason arrière"""
        self.rear_obstacle_distance = msg.range
    
    def line_detection_callback(self, msg):
        """Callback pour le statut de détection de ligne"""
        self.line_detected = msg.data
        if self.line_detected:
            self.last_line_detection_time = time.time()
    
    def is_obstacle_detected(self):
        """Vérifie s'il y a un obstacle dans la direction de navigation"""
        if self.went_forward:
            return self.front_obstacle_distance < self.obstacle_threshold
        else:
            return self.rear_obstacle_distance < self.obstacle_threshold
    
    def get_obstacle_distance(self):
        """Retourne la distance de l'obstacle dans la direction de navigation"""
        if self.went_forward:
            return self.front_obstacle_distance
        else:
            return self.rear_obstacle_distance

    def bottom_camera_callback(self, data):
        """Callback pour la caméra bottom (fixe sous le châssis) - SEULE caméra pour ArUcos"""
        # TOUJOURS traiter les images de la caméra bottom, même si pas en navigation
        # Log pour confirmer réception d'images
        self.get_logger().info(f'Bottom camera callback - is_navigating: {self.is_navigating}', throttle_duration_sec=5.0)
        
        # Détecter les ArUcos même en dehors de la navigation pour debug
        self._detect_aruco(data, self.bottom_aruco_pub, "BOTTOM")

    def _detect_aruco(self, image_msg, publisher, camera_name):
        """Détecte les marqueurs ArUco dans l'image de la caméra BOTTOM uniquement"""
        try:
            self.get_logger().info(f'{camera_name}: Processing image for ArUco detection', throttle_duration_sec=5.0)
            
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            display_image = cv_image.copy()
            
            # Utiliser toute l'image pour la caméra bottom (pas de crop)
            height, width = cv_image.shape[:2]
            self.get_logger().info(f'{camera_name}: Image size: {width}x{height}', throttle_duration_sec=10.0)
            
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            self.get_logger().info(f'{camera_name}: ArUco detection - Found: {len(ids) if ids is not None else 0}, Rejected: {len(rejected)}', throttle_duration_sec=2.0)
            
            if ids is not None and len(ids) > 0:
                detected_id = int(ids[0][0])
                
                # Calculer le centre du marqueur
                marker_corners = corners[0][0]
                marker_center_x = int(np.mean(marker_corners[:, 0]))
                marker_center_y = int(np.mean(marker_corners[:, 1]))
                
                # Sauvegarder la position pour le centrage
                self.aruco_center_x = marker_center_x
                self.aruco_center_y = marker_center_y
                self.image_center_x = width // 2
                self.image_center_y = height // 2
                
                # Calculer l'offset par rapport au centre
                offset_x = marker_center_x - self.image_center_x
                offset_y = marker_center_y - self.image_center_y
                
                # Dessiner les marqueurs détectés
                cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                
                # Dessiner le centre de l'image et du marqueur
                cv2.circle(display_image, (self.image_center_x, self.image_center_y), 10, (255, 0, 0), 2)  # Centre image (bleu)
                cv2.circle(display_image, (marker_center_x, marker_center_y), 10, (0, 255, 0), 2)  # Centre marqueur (vert)
                cv2.line(display_image, (self.image_center_x, self.image_center_y), (marker_center_x, marker_center_y), (0, 0, 255), 2)
                
                # Afficher l'ID et le nombre de détections
                cv2.putText(display_image, f'{camera_name} Camera - ArUco Detection', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 255, 0), 2)
                cv2.putText(display_image, f'ArUco ID: {detected_id}', 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 255, 0), 2)
                cv2.putText(display_image, f'Center: ({marker_center_x}, {marker_center_y})', 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 255, 0), 2)
                cv2.putText(display_image, f'Offset: ({offset_x}, {offset_y})', 
                           (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 255, 255), 2)
                cv2.putText(display_image, f'Detections: {self.aruco_detection_count}/{self.required_detections}', 
                           (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 255, 0), 2)
                
                # Détection avec filtrage - détecte dès que le marqueur est visible
                if detected_id == self.current_aruco_id:
                    self.aruco_detection_count += 1
                else:
                    self.current_aruco_id = detected_id
                    self.aruco_detection_count = 1
                
                if self.aruco_detection_count >= self.required_detections:
                    self.get_logger().info(
                        f'📍 ArUco {self.current_aruco_id} détecté par {camera_name}',
                        throttle_duration_sec=1.0
                    )
            else:
                # Pas de marqueur détecté
                cv2.putText(display_image, f'{camera_name} Camera - NO ArUco', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 0, 255), 2)
                
                if self.aruco_detection_count > 0:
                    self.aruco_detection_count -= 1
            
            # Publier l'image avec les détections
            detection_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
            publisher.publish(detection_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Erreur détection ArUco ({camera_name}): {str(e)}')

    def goal_callback(self, goal_request):
        """Accepte ou rejette les demandes de navigation"""
        target_id = goal_request.target_aruco_id
        
        self.get_logger().info(f'📥 Demande de navigation vers ArUco {target_id}')
        
        # Validation du numéro ArUco
        if target_id < 0 or target_id > 49:  # DICT_4X4_50 va de 0 à 49
            self.get_logger().warn(f'❌ Numéro ArUco invalide: {target_id}')
            return GoalResponse.REJECT
        
        # Rejeter si déjà en navigation
        if self.is_navigating:
            self.get_logger().warn('❌ Navigation déjà en cours!')
            return GoalResponse.REJECT
        
        self.get_logger().info('✅ Objectif accepté - Démarrage de la navigation!')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Permet l'annulation de la navigation"""
        self.get_logger().info('🛑 Demande d\'annulation reçue')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Exécute la navigation vers le marqueur ArUco avec machine à états"""
        self.target_id = goal_handle.request.target_aruco_id
        self.is_navigating = True
        self.start_time = time.time()
        self.current_state = NavigationState.NAVIGATE_TO_TARGET
        
        feedback_msg = NavigateToAruco.Feedback()
        color_choice = goal_handle.request.color_choice
        
        # Déterminer la direction de rotation et couleur
        # 'l' = rotation à DROITE pour suivre ligne ROUGE
        # 'r' = rotation à GAUCHE pour suivre ligne VERTE
        if color_choice == 'l':
            self.rotation_direction = 'right'  # Rotation à droite
            target_color = 'red'
        elif color_choice == 'r':
            self.rotation_direction = 'left'   # Rotation à gauche
            target_color = 'green'
        else:
            self.rotation_direction = None
            target_color = 'blue'
        
        self.get_logger().info(f'⚡ Navigation vers ArUco {self.target_id}')
        self.get_logger().info(f'📋 États: NAVIGATE→ROTATE→FOLLOW_COLOR→OBSTACLE→RETURN→ROTATE→RETURN_BASE')
        
        # ÉTAT 1: NAVIGATE_TO_TARGET - Recherche position de départ
        self.get_logger().info('🔵 ÉTAT 1: NAVIGATE_TO_TARGET - Suivi ligne bleue')
        self.current_state = NavigationState.NAVIGATE_TO_TARGET
        self._set_line_color('blue')
        self._enable_movement(True)
        self._set_direction(True)
        
        initial_wait_time = 0
        max_initial_wait = 30.0
        
        while self.current_aruco_id is None and initial_wait_time < max_initial_wait:
            if goal_handle.is_cancel_requested:
                return self._handle_cancellation(goal_handle)
            time.sleep(0.5)
            initial_wait_time += 0.5
            feedback_msg.current_aruco_id = 0
            feedback_msg.current_direction = 'Recherche position...'
            feedback_msg.elapsed_time = time.time() - self.start_time
            feedback_msg.status_message = 'ÉTAT 1: Détection position départ'
            feedback_msg.obstacle_detected = False
            feedback_msg.obstacle_distance = 0.0
            goal_handle.publish_feedback(feedback_msg)
        
        if self.current_aruco_id is None:
            self.get_logger().warn('⚠️ Aucun ArUco détecté')
            return self._return_failure(goal_handle)
        
        starting_aruco = self.current_aruco_id
        self.get_logger().info(f'📍 Position départ: ArUco {starting_aruco}')
        
        # Déterminer la direction
        if self.target_id < starting_aruco:
            self.went_forward = False
            direction_text = "ARRIÈRE"
            self.get_logger().info(f'⬅️ Marche arrière vers {self.target_id}')
            self._set_direction(False)
        elif self.target_id > starting_aruco:
            self.went_forward = True
            direction_text = "AVANT"
            self.get_logger().info(f'➡️ Marche avant vers {self.target_id}')
            self._set_direction(True)
        else:
            self.get_logger().info('✅ Déjà à la cible!')
            return self._return_success(goal_handle, starting_aruco)
        
        # Navigation jusqu'à la cible
        self.get_logger().info(f'🚀 Navigation vers ArUco {self.target_id}...')
        
        segment_timeout = 60.0
        last_aruco_change_time = time.time()
        last_detected_aruco = starting_aruco
        last_feedback_time = time.time()
        feedback_interval = 1.0
        last_movement_check = time.time()
        movement_check_interval = 3.0
        obstacle_wait_start = None
        
        # Boucle de navigation jusqu'à la cible
        while self.current_state == NavigationState.NAVIGATE_TO_TARGET:
            if goal_handle.is_cancel_requested:
                return self._handle_cancellation(goal_handle)
            
            # Gérer les obstacles
            obstacle_present = self.is_obstacle_detected()
            obstacle_distance = self.get_obstacle_distance()
            
            if obstacle_present:
                if obstacle_wait_start is None:
                    obstacle_wait_start = time.time()
                    self.get_logger().warn(f'🚨 OBSTACLE: {obstacle_distance:.2f}m')
                wait_time = time.time() - obstacle_wait_start
                feedback_msg.obstacle_detected = True
                feedback_msg.obstacle_distance = obstacle_distance
                feedback_msg.status_message = f'ÉTAT 1: OBSTACLE à {obstacle_distance:.2f}m - Attente: {wait_time:.1f}s'
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)
                continue
            else:
                if obstacle_wait_start is not None:
                    self.get_logger().info(f'✅ Obstacle enlevé')
                    obstacle_wait_start = None
            
            # Mettre à jour détection
            current_id = self.current_aruco_id if self.current_aruco_id is not None else 0
            if current_id > 0 and current_id != last_detected_aruco:
                last_aruco_change_time = time.time()
                last_detected_aruco = current_id
                self.get_logger().info(f'🔄 ArUco {current_id} détecté')
            
            # Timeout
            if time.time() - last_aruco_change_time > segment_timeout:
                self.get_logger().warn(f'⏱️ Timeout')
                return self._return_failure(goal_handle)
            
            # Réactiver mouvement
            if time.time() - last_movement_check >= movement_check_interval:
                self._enable_movement(True)
                self._set_direction(self.went_forward)
                last_movement_check = time.time()
            
            # Vérifier si cible atteinte
            if (self.current_aruco_id == self.target_id and 
                self.aruco_detection_count >= self.required_detections):
                self.get_logger().info(f'🎯 Cible {self.target_id} atteinte!')
                time.sleep(0.5)
                
                # Passer à l'état suivant si color_choice spécifié
                if color_choice and color_choice in ['l', 'r']:
                    self.current_state = NavigationState.ROTATE_AT_TARGET
                    break
                else:
                    # Pas de color_choice, retourner directement ou terminer
                    if goal_handle.request.return_to_zero and self.target_id != 0:
                        return_result = self._navigate_to_zero(goal_handle, feedback_msg)
                        if return_result:
                            return return_result
                    return self._return_success(goal_handle, self.target_id, goal_handle.request.return_to_zero)
            
            # Feedback
            if time.time() - last_feedback_time >= feedback_interval:
                feedback_msg.obstacle_detected = False
                feedback_msg.obstacle_distance = obstacle_distance
                feedback_msg.current_aruco_id = current_id
                feedback_msg.current_direction = direction_text
                feedback_msg.elapsed_time = time.time() - self.start_time
                if current_id > 0:
                    distance = abs(self.target_id - current_id)
                    feedback_msg.status_message = f'ÉTAT 1: ArUco {current_id}→{self.target_id} (dist:{distance})'
                else:
                    feedback_msg.status_message = 'ÉTAT 1: Suivi ligne bleue...'
                goal_handle.publish_feedback(feedback_msg)
                last_feedback_time = time.time()
            
            time.sleep(0.1)
        
        # ÉTAT 2: ROTATE_AT_TARGET - Rotation à la cible
        if self.current_state == NavigationState.ROTATE_AT_TARGET:
            # Si pas de choix de couleur ('l' ou 'r'), passer directement à la fin
            if color_choice not in ['l', 'r']:
                self.get_logger().info('✅ ArUco cible atteint, pas de rotation demandée')
                self.current_state = NavigationState.RETURN_TO_BASE
            else:
                self.get_logger().info(f'🔄 ÉTAT 2: ROTATE_AT_TARGET - Rotation 90° {self.rotation_direction}')
                
                # Désactiver le line follower
                self._enable_movement(False)
                time.sleep(0.5)
                
                # Rotation de 90 degrés (environ 2.5 secondes à vitesse angulaire de 0.6 rad/s)
                # Pour rotation droite (l) ou gauche (r)
                self._rotate_robot(self.rotation_direction, duration=7.0)
                
                self.current_state = NavigationState.FOLLOW_COLOR_TO_OBSTACLE
        
        # ÉTAT 3: FOLLOW_COLOR_TO_OBSTACLE - Suivi couleur jusqu'à perte de ligne
        if self.current_state == NavigationState.FOLLOW_COLOR_TO_OBSTACLE:
            self.get_logger().info(f'🎨 ÉTAT 3: FOLLOW_COLOR_TO_OBSTACLE - Suivi ligne {target_color.upper()}')
            self._set_line_color(target_color)
            self._set_direction(True)
            self.went_forward = True
            self._enable_movement(True)
            
            line_loss_timeout = 5.0  # 5 secondes sans ligne
            overall_timeout = 60.0
            start_wait = time.time()
            
            while True:
                if goal_handle.is_cancel_requested:
                    return self._handle_cancellation(goal_handle)
                if time.time() - start_wait > overall_timeout:
                    self.get_logger().warn('⏱️ Timeout général')
                    break
                
                # Vérifier si la ligne n'est plus détectée depuis 5 secondes
                time_since_line = time.time() - self.last_line_detection_time
                if time_since_line > line_loss_timeout:
                    self.get_logger().info(f'🛑 Ligne {target_color} perdue depuis {time_since_line:.1f}s')
                    break
                
                feedback_msg.status_message = f'ÉTAT 3: Suivi {target_color} (ligne: {"OK" if self.line_detected else "PERDUE"})'
                feedback_msg.elapsed_time = time.time() - self.start_time
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)
            
            self.current_state = NavigationState.OBSTACLE_DETECTED
        
        # ÉTAT 4: OBSTACLE_DETECTED - Arrêt au bout de la ligne
        if self.current_state == NavigationState.OBSTACLE_DETECTED:
            self.get_logger().info('🛑 ÉTAT 4: OBSTACLE_DETECTED - Fin de ligne colorée')
            self._enable_movement(False)
            time.sleep(1.0)
            self.get_logger().info('🔄 Changement de direction - utilisation caméra arrière')
            self.current_state = NavigationState.RETURN_ON_COLOR
        
        # ÉTAT 5: RETURN_ON_COLOR - Retour arrière sur couleur jusqu'à perte de ligne
        if self.current_state == NavigationState.RETURN_ON_COLOR:
            self.get_logger().info(f'⬅️ ÉTAT 5: RETURN_ON_COLOR - Retour arrière sur {target_color} (caméra arrière)')
            self._set_direction(False)  # Marche arrière - active caméra arrière
            self.went_forward = False
            time.sleep(0.5)  # Attendre que la direction soit changée
            self._enable_movement(True)
            
            # Reset line detection timer - attendre un peu que la ligne soit détectée
            time.sleep(1.0)
            self.last_line_detection_time = time.time()
            
            # Attendre que la ligne soit détectée (on devrait être sur la ligne)
            wait_for_line_start = time.time()
            while not self.line_detected and (time.time() - wait_for_line_start) < 3.0:
                time.sleep(0.1)
            
            if self.line_detected:
                self.get_logger().info(f'✅ Ligne {target_color} détectée - début du retour arrière')
            else:
                self.get_logger().warn(f'⚠️ Ligne {target_color} non détectée - retour quand même')
            
            return_timeout = 60.0
            return_start = time.time()
            line_was_detected = False
            
            while True:
                if goal_handle.is_cancel_requested:
                    return self._handle_cancellation(goal_handle)
                if time.time() - return_start > return_timeout:
                    self.get_logger().warn('⏱️ Timeout retour')
                    break
                
                # Marquer si on a détecté la ligne au moins une fois
                if self.line_detected:
                    line_was_detected = True
                
                # Attendre que la ligne soit perdue (si elle a été détectée)
                if line_was_detected and not self.line_detected:
                    self.get_logger().info(f'✅ Ligne {target_color} n\'est plus détectée - fin du retour arrière')
                    time.sleep(0.5)  # Attendre un peu pour être sûr
                    break
                
                feedback_msg.status_message = f'ÉTAT 5: Retour arrière sur {target_color} (ligne: {"OK" if self.line_detected else "NON"})'
                feedback_msg.elapsed_time = time.time() - self.start_time
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)
            
            self._enable_movement(False)
            time.sleep(0.5)
            self.current_state = NavigationState.ROTATE_AT_MARKER
        
        # ÉTAT 6: ROTATE_AT_MARKER - Rotation pour retrouver ligne bleue
        if self.current_state == NavigationState.ROTATE_AT_MARKER:
            # Rotation basée sur la couleur suivie: left pour rouge, right pour vert
            if self.rotation_direction is not None:
                if target_color == 'red':
                    rotation = 'left'
                    self.get_logger().info(f'🔄 ÉTAT 6: Rotation GAUCHE (rouge) vers ligne bleue')
                elif target_color == 'green':
                    rotation = 'right'
                    self.get_logger().info(f'🔄 ÉTAT 6: Rotation DROITE (vert) vers ligne bleue')
                else:
                    rotation = 'left'
                    self.get_logger().info(f'🔄 ÉTAT 6: Rotation par défaut')
                
                self._rotate_robot(rotation, duration=5.0)
            else:
                self.get_logger().info('✅ ÉTAT 6: Pas de rotation nécessaire')
            
            self._set_line_color('blue')
            self.current_state = NavigationState.RETURN_TO_BASE
        
        # ÉTAT 7: RETURN_TO_BASE - Retour sur ligne bleue
        if self.current_state == NavigationState.RETURN_TO_BASE:
            if goal_handle.request.return_to_zero and self.target_id != 0:
                self.get_logger().info('🏠 ÉTAT 7: RETURN_TO_BASE - Retour à ArUco 0')
                return_result = self._navigate_to_zero(goal_handle, feedback_msg)
                if return_result:
                    return return_result
            else:
                self.get_logger().info('✅ Mission terminée sans retour base')
        
        return self._return_success(goal_handle, self.target_id, goal_handle.request.return_to_zero)

    def _enable_movement(self, enable):
        """Active ou désactive le mouvement du robot"""
        if not self.enable_movement_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service enable_movement non disponible')
            return False
        
        request = SetBool.Request()
        request.data = enable
        future = self.enable_movement_client.call_async(request)
        return True

    def _set_direction(self, forward):
        """Définit la direction du robot (avant/arrière)"""
        if not self.set_direction_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service set_forward_direction non disponible')
            return False
        
        request = SetBool.Request()
        request.data = forward
        future = self.set_direction_client.call_async(request)
        return True
    
    def _set_line_color(self, color):
        """Change la couleur de ligne à suivre (blue, red, green)"""
        msg = String()
        msg.data = color
        self.color_change_publisher.publish(msg)
        time.sleep(0.5)  # Give time for the color change to take effect
        self.get_logger().info(f'🎨 Changement de couleur vers: {color.upper()}')
    
    def send_velocity(self, linear=0.0, angular=0.0):
        """Envoie une commande de vitesse au robot
        Args:
            linear: vitesse linéaire normalisée (-1.0 à 1.0)
            angular: vitesse angulaire normalisée (-1.0 à 1.0)
        """
        twist = Twist()
        twist.linear.x = linear * self.linear_speed
        twist.angular.z = angular * self.angular_speed
        # Publier plusieurs fois pour garantir la réception
        for _ in range(5):
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.01)
    
    def _rotate_robot(self, direction, duration=3.5):
        """Fait tourner le robot à gauche ou à droite de 90 degrés
        Args:
            direction: 'left' ou 'right'
            duration: durée de la rotation en secondes (3.5s pour 90° complet)
        """
        # Arrêter le line follower temporairement
        self._enable_movement(False)
        time.sleep(0.5)
        
        # Publier commande de rotation pour 90 degrés
        # Vitesse angulaire normalisée: 0.5 pour gauche, -0.5 pour droite
        angular_value = 0.5 if direction == 'left' else -0.5
        
        rotation_start = time.time()
        while time.time() - rotation_start < duration:
            # Publier continuellement pendant la rotation
            self.send_velocity(linear=0.0, angular=angular_value)
            time.sleep(0.05)  # Publier à 20Hz
        
        # Arrêter la rotation en publiant plusieurs fois
        for _ in range(10):
            self.send_velocity(linear=0.0, angular=0.0)
            time.sleep(0.05)
        
        # Réactiver le line follower
        self._enable_movement(True)
        time.sleep(0.3)
        self.get_logger().info(f'🔄 Rotation 90° {direction} terminée')

    def _navigate_to_zero(self, goal_handle, feedback_msg):
        """Navigue vers ArUco 0 après avoir atteint la cible"""
        self.get_logger().info('🔙 Démarrage du retour vers ArUco 0...')
        
        # Réinitialiser pour la navigation de retour
        return_target = 0
        current_position = self.current_aruco_id
        
        # Déterminer la direction pour retourner à 0
        if return_target < current_position:
            # Marche arrière
            self.went_forward = False
            direction_text = "ARRIÈRE (rear camera)"
            self.get_logger().info(f'⬅️ ArUco 0 est avant, marche arrière')
            self._set_direction(False)
        else:
            # Marche avant
            self.went_forward = True
            direction_text = "AVANT (front camera)"
            self.get_logger().info(f'➡️ ArUco 0 est après, marche avant')
            self._set_direction(True)
        
        # Navigation vers 0
        segment_timeout = 60.0
        last_aruco_change_time = time.time()
        last_detected_aruco = current_position
        last_feedback_time = time.time()
        feedback_interval = 1.0
        last_movement_check = time.time()
        movement_check_interval = 3.0
        obstacle_wait_start = None
        
        while True:
            if goal_handle.is_cancel_requested:
                return self._handle_cancellation(goal_handle)
            
            # Vérifier obstacles
            obstacle_present = self.is_obstacle_detected()
            obstacle_distance = self.get_obstacle_distance()
            
            if obstacle_present:
                if obstacle_wait_start is None:
                    obstacle_wait_start = time.time()
                    direction = "AVANT" if self.went_forward else "ARRIÈRE"
                    self.get_logger().warn(f'🚨 OBSTACLE DÉTECTÉ {direction}: {obstacle_distance:.2f}m - EN ATTENTE...')
                
                wait_time = time.time() - obstacle_wait_start
                feedback_msg.obstacle_detected = True
                feedback_msg.obstacle_distance = obstacle_distance
                feedback_msg.status_message = f'⚠️ OBSTACLE à {obstacle_distance:.2f}m - Attente: {wait_time:.1f}s (Retour à 0)'
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)
                continue
            else:
                if obstacle_wait_start is not None:
                    wait_duration = time.time() - obstacle_wait_start
                    self.get_logger().info(f'✅ Obstacle enlevé après {wait_duration:.1f}s - Reprise du retour')
                    obstacle_wait_start = None
            
            # Vérifier nouveau marqueur
            current_id = self.current_aruco_id if self.current_aruco_id is not None else 0
            if current_id > 0 and current_id != last_detected_aruco:
                last_aruco_change_time = time.time()
                last_detected_aruco = current_id
                self.get_logger().info(f'🔄 Nouveau marqueur détecté: ArUco {current_id}')
            
            # Timeout
            time_since_last_change = time.time() - last_aruco_change_time
            if time_since_last_change > segment_timeout:
                self.get_logger().warn(f'⏱️ Timeout pendant retour à 0')
                return self._return_failure(goal_handle)
            
            # Réactiver mouvement
            if time.time() - last_movement_check >= movement_check_interval:
                self._enable_movement(True)
                self._set_direction(self.went_forward)
                last_movement_check = time.time()
            
            # Vérifier si on a atteint 0
            if (self.current_aruco_id == 0 and 
                self.aruco_detection_count >= self.required_detections):
                self.get_logger().info(f'🏁 ArUco 0 atteint! Retour terminé!')
                time.sleep(0.5)
                return None  # Retourner None pour indiquer succès
            
            # Feedback
            if time.time() - last_feedback_time >= feedback_interval:
                feedback_msg.obstacle_detected = False
                feedback_msg.obstacle_distance = obstacle_distance
                feedback_msg.current_aruco_id = current_id
                feedback_msg.current_direction = direction_text + " (Retour)"
                feedback_msg.elapsed_time = time.time() - self.start_time
                
                if current_id > 0:
                    distance = abs(0 - current_id)
                    feedback_msg.status_message = f'🔙 Retour: ArUco {current_id} → 0 (distance: {distance})'
                else:
                    feedback_msg.status_message = '🔙 Retour vers ArUco 0...'
                
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'📊 {feedback_msg.status_message}')
                last_feedback_time = time.time()
            
            time.sleep(0.1)

    def _handle_cancellation(self, goal_handle):
        """Gère l'annulation de la navigation"""
        self._enable_movement(False)
        goal_handle.canceled()
        
        result = NavigateToAruco.Result()
        result.success = False
        result.final_aruco_id = self.current_aruco_id if self.current_aruco_id else 0
        result.went_forward = self.went_forward
        result.navigation_time = time.time() - self.start_time
        result.distance_traveled = 0.0
        
        self.is_navigating = False
        self.get_logger().info('🛑 Navigation annulée')
        
        return result

    def _return_success(self, goal_handle, final_id, returned_to_zero=False):
        """Retourne un résultat de succès"""
        self._enable_movement(False)
        goal_handle.succeed()
        
        result = NavigateToAruco.Result()
        result.success = True
        result.final_aruco_id = final_id
        result.went_forward = self.went_forward
        result.navigation_time = time.time() - self.start_time
        result.distance_traveled = 0.0  # TODO: calculer distance réelle
        result.returned_to_zero = returned_to_zero
        
        self.is_navigating = False
        
        self.get_logger().info('=================================')
        self.get_logger().info('✅ NAVIGATION RÉUSSIE!')
        self.get_logger().info(f'   ArUco final: {final_id}')
        self.get_logger().info(f'   Direction: {"AVANT" if self.went_forward else "ARRIÈRE"}')
        self.get_logger().info(f'   Temps: {result.navigation_time:.1f}s')
        if returned_to_zero:
            self.get_logger().info('   🔄 Retour à ArUco 0 effectué')
        self.get_logger().info('=================================')
        
        return result

    def _return_failure(self, goal_handle):
        """Retourne un résultat d'échec"""
        self._enable_movement(False)
        goal_handle.abort()
        
        result = NavigateToAruco.Result()
        result.success = False
        result.final_aruco_id = self.current_aruco_id if self.current_aruco_id else 0
        result.went_forward = self.went_forward
        result.navigation_time = time.time() - self.start_time
        result.distance_traveled = 0.0
        
        self.is_navigating = False
        
        self.get_logger().error('=================================')
        self.get_logger().error('❌ NAVIGATION ÉCHOUÉE')
        self.get_logger().error(f'   Dernier ArUco: {result.final_aruco_id}')
        self.get_logger().error(f'   Temps: {result.navigation_time:.1f}s')
        self.get_logger().error('=================================')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    
    server = ArucoNavigationServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        server.get_logger().info('Arrêt du serveur...')
    
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
