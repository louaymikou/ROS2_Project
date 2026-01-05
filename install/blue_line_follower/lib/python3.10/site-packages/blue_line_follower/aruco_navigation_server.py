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
from cv_bridge import CvBridge
import cv2


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
        self.required_detections = 2  # Nombre de détections consécutives requises (réduit pour plus de réactivité)
        
        # Obstacle detection
        self.front_obstacle_distance = float('inf')
        self.rear_obstacle_distance = float('inf')
        self.obstacle_threshold = 0.3  # 30cm
        
        # Subscribe to both cameras
        self.front_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.front_camera_callback,
            10)
        
        self.rear_subscription = self.create_subscription(
            Image,
            '/rear_camera/image_raw',
            self.rear_camera_callback,
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
        
        # Service clients pour contrôler le robot
        self.enable_movement_client = self.create_client(SetBool, 'enable_movement')
        self.set_direction_client = self.create_client(SetBool, 'set_forward_direction')
        
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
        
        self.get_logger().info('=================================')
        self.get_logger().info('🎯 ArUco Navigation Server Ready')
        self.get_logger().info('   Action: /navigate_to_aruco')
        self.get_logger().info(f'   Obstacle detection: {self.obstacle_threshold}m')
        self.get_logger().info('=================================')

    def front_ultrasonic_callback(self, msg):
        """Callback pour le capteur ultrason avant"""
        self.front_obstacle_distance = msg.range

    def rear_ultrasonic_callback(self, msg):
        """Callback pour le capteur ultrason arrière"""
        self.rear_obstacle_distance = msg.range
    
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

    def front_camera_callback(self, data):
        """Callback pour la caméra avant"""
        if self.is_navigating and self.went_forward:
            self._detect_aruco(data)

    def rear_camera_callback(self, data):
        """Callback pour la caméra arrière"""
        if self.is_navigating and not self.went_forward:
            self._detect_aruco(data)

    def _detect_aruco(self, image_msg):
        """Détecte les marqueurs ArUco dans l'image (seulement en bas)"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Only detect ArUco at the bottom 15% of the camera image
            height, width, _ = cv_image.shape
            aruco_detect_start = int(height * 0.85)  # Bottom 15%
            aruco_roi = cv_image[aruco_detect_start:height, 0:width]
            
            # Convert to grayscale and detect
            gray = cv2.cvtColor(aruco_roi, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None and len(ids) > 0:
                detected_id = int(ids[0][0])
                
                # Détection avec filtrage
                if detected_id == self.current_aruco_id:
                    self.aruco_detection_count += 1
                else:
                    self.current_aruco_id = detected_id
                    self.aruco_detection_count = 1
                
                if self.aruco_detection_count >= self.required_detections:
                    self.get_logger().info(
                        f'📍 ArUco {self.current_aruco_id} détecté de manière stable',
                        throttle_duration_sec=2.0
                    )
            else:
                # Pas de marqueur détecté
                if self.aruco_detection_count > 0:
                    self.aruco_detection_count -= 1
                    
        except Exception as e:
            self.get_logger().error(f'Erreur détection ArUco: {str(e)}')

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
        """Exécute la navigation vers le marqueur ArUco"""
        self.target_id = goal_handle.request.target_aruco_id
        self.is_navigating = True
        self.start_time = time.time()
        
        feedback_msg = NavigateToAruco.Feedback()
        
        self.get_logger().info(f'⚡ Navigation vers ArUco {self.target_id}')
        
        # Étape 1: Attendre d'avoir une position de départ (ArUco actuel)
        self.get_logger().info('🔍 Recherche de la position actuelle...')
        
        # Activer le mouvement pour commencer à chercher
        self._enable_movement(True)
        self._set_direction(True)  # Commencer en marche avant pour détecter
        
        initial_wait_time = 0
        max_initial_wait = 10.0  # 10 secondes max pour détecter position initiale
        
        while self.current_aruco_id is None and initial_wait_time < max_initial_wait:
            if goal_handle.is_cancel_requested:
                return self._handle_cancellation(goal_handle)
            
            time.sleep(0.5)
            initial_wait_time += 0.5
            
            feedback_msg.current_aruco_id = 0
            feedback_msg.current_direction = 'Recherche position initiale...'
            feedback_msg.elapsed_time = time.time() - self.start_time
            feedback_msg.status_message = 'Détection de la position de départ'
            feedback_msg.obstacle_detected = False
            feedback_msg.obstacle_distance = 0.0
            goal_handle.publish_feedback(feedback_msg)
        
        if self.current_aruco_id is None:
            self.get_logger().warn('⚠️ Aucun ArUco détecté au départ, navigation impossible')
            return self._return_failure(goal_handle)
        
        starting_aruco = self.current_aruco_id
        self.get_logger().info(f'📍 Position de départ: ArUco {starting_aruco}')
        
        # Étape 2: Déterminer la direction
        if self.target_id < starting_aruco:
            # Le marqueur cible est avant nous -> marche arrière
            self.went_forward = False
            direction_text = "ARRIÈRE (rear camera)"
            self.get_logger().info(f'⬅️ ArUco {self.target_id} est avant, marche arrière')
            self._set_direction(False)
        elif self.target_id > starting_aruco:
            # Le marqueur cible est après nous -> marche avant
            self.went_forward = True
            direction_text = "AVANT (front camera)"
            self.get_logger().info(f'➡️ ArUco {self.target_id} est après, marche avant')
            self._set_direction(True)
        else:
            # Nous sommes déjà au bon marqueur!
            self.get_logger().info('✅ Déjà à la position cible!')
            return self._return_success(goal_handle, starting_aruco)
        
        # Étape 3: Naviguer jusqu'au marqueur cible
        self.get_logger().info(f'🚀 Navigation en cours vers ArUco {self.target_id}...')
        
        segment_timeout = 60.0  # 60 secondes max sans détecter de nouveau marqueur
        last_aruco_change_time = time.time()  # Timer réinitialisé à chaque nouveau marqueur
        last_detected_aruco = starting_aruco  # Dernier ArUco détecté
        
        last_feedback_time = time.time()
        feedback_interval = 1.0  # Envoyer feedback chaque seconde
        last_movement_check = time.time()
        movement_check_interval = 3.0  # Réactiver le mouvement toutes les 3 secondes
        obstacle_wait_start = None  # Temps où l'obstacle a été détecté
        
        while True:
            # Vérifier l'annulation
            if goal_handle.is_cancel_requested:
                return self._handle_cancellation(goal_handle)
            
            # Vérifier la présence d'obstacles
            obstacle_present = self.is_obstacle_detected()
            obstacle_distance = self.get_obstacle_distance()
            
            if obstacle_present:
                if obstacle_wait_start is None:
                    obstacle_wait_start = time.time()
                    direction = "AVANT" if self.went_forward else "ARRIÈRE"
                    self.get_logger().warn(f'🚨 OBSTACLE DÉTECTÉ {direction}: {obstacle_distance:.2f}m - EN ATTENTE...')
                
                # Envoyer feedback d'obstacle
                wait_time = time.time() - obstacle_wait_start
                feedback_msg.obstacle_detected = True
                feedback_msg.obstacle_distance = obstacle_distance
                feedback_msg.current_aruco_id = current_id if 'current_id' in locals() else 0
                feedback_msg.current_direction = direction_text
                feedback_msg.elapsed_time = time.time() - self.start_time
                feedback_msg.status_message = f'⚠️ OBSTACLE à {obstacle_distance:.2f}m - Attente: {wait_time:.1f}s'
                goal_handle.publish_feedback(feedback_msg)
                
                # Le robot attend que l'obstacle soit enlevé
                time.sleep(0.5)
                continue
            else:
                # Plus d'obstacle
                if obstacle_wait_start is not None:
                    wait_duration = time.time() - obstacle_wait_start
                    self.get_logger().info(f'✅ Obstacle enlevé après {wait_duration:.1f}s - Reprise de la navigation')
                    obstacle_wait_start = None
            
            # Réinitialiser le timeout si un nouveau marqueur est détecté
            current_id = self.current_aruco_id if self.current_aruco_id is not None else 0
            if current_id > 0 and current_id != last_detected_aruco:
                last_aruco_change_time = time.time()
                last_detected_aruco = current_id
                self.get_logger().info(f'🔄 Nouveau marqueur détecté: ArUco {current_id} - Timer réinitialisé')
            
            # Vérifier le timeout par segment (temps sans nouveau marqueur)
            time_since_last_change = time.time() - last_aruco_change_time
            if time_since_last_change > segment_timeout:
                self.get_logger().warn(f'⏱️ Timeout: Aucun nouveau marqueur depuis {segment_timeout}s')
                return self._return_failure(goal_handle)
            
            # Réactiver le mouvement périodiquement pour s'assurer qu'il reste actif
            if time.time() - last_movement_check >= movement_check_interval:
                self._enable_movement(True)
                self._set_direction(self.went_forward)  # Réappliquer la direction aussi
                last_movement_check = time.time()
                self.get_logger().debug('🔄 Réactivation du mouvement et direction')
            
            # Vérifier si on a atteint la cible
            if (self.current_aruco_id == self.target_id and 
                self.aruco_detection_count >= self.required_detections):
                self.get_logger().info(f'🎯 Marqueur cible {self.target_id} atteint!')
                # Attendre un peu pour stabiliser
                time.sleep(0.5)
                
                # Vérifier si on doit retourner à ArUco 0
                if goal_handle.request.return_to_zero and self.target_id != 0:
                    self.get_logger().info('🔄 Retour à ArUco 0...')
                    return_result = self._navigate_to_zero(goal_handle, feedback_msg)
                    if return_result:
                        return return_result
                
                return self._return_success(goal_handle, self.target_id, goal_handle.request.return_to_zero)
            
            # Envoyer feedback périodiquement
            if time.time() - last_feedback_time >= feedback_interval:
                feedback_msg.obstacle_detected = False
                feedback_msg.obstacle_distance = obstacle_distance
                feedback_msg.current_aruco_id = current_id
                feedback_msg.current_direction = direction_text
                feedback_msg.elapsed_time = time.time() - self.start_time
                
                if current_id > 0:
                    distance = abs(self.target_id - current_id)
                    time_left = segment_timeout - time_since_last_change
                    feedback_msg.status_message = f'ArUco {current_id} → {self.target_id} (distance: {distance}) [timeout: {time_left:.1f}s] [détections: {self.aruco_detection_count}/{self.required_detections}]'
                else:
                    feedback_msg.status_message = 'Suivi de la ligne...'
                
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'📊 {feedback_msg.status_message}')
                last_feedback_time = time.time()
            
            time.sleep(0.1)

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
