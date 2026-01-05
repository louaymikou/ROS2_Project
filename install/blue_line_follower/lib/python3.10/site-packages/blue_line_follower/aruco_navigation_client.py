#!/usr/bin/env python3
"""
ArUco Navigation Client
========================
Client pour tester l'action de navigation ArUco.
"""

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.node import Node
from custom_interfaces.action import NavigateToAruco


class ArucoNavigationClient(Node):
    """
    Client qui demande la navigation vers un marqueur ArUco.
    """

    def __init__(self):
        super().__init__('aruco_navigation_client')
        
        # Créer le client d'action
        self.action_client = ActionClient(
            self,
            NavigateToAruco,
            'navigate_to_aruco'
        )

    def send_goal(self, target_id):
        """
        Demande de naviguer vers un marqueur ArUco spécifique.
        """
        self.get_logger().info(f'🎯 Demande de navigation vers ArUco {target_id}...')
        
        # Attendre le serveur
        self.get_logger().info('⏳ Attente du serveur d\'action...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Serveur d\'action non disponible!')
            return False
        
        # Créer l'objectif
        goal_msg = NavigateToAruco.Goal()
        goal_msg.target_aruco_id = target_id
        
        # Envoyer l'objectif de manière asynchrone
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Enregistrer les callbacks
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True

    def goal_response_callback(self, future):
        """
        Appelé quand le serveur accepte/rejette notre objectif.
        """
        self.goal_handle: ClientGoalHandle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('❌ Demande de navigation rejetée!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('✅ Demande acceptée!')
        self.get_logger().info('🚀 Navigation en cours...')
        self.get_logger().info('')
        
        # Obtenir le résultat quand terminé
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Appelé quand la navigation est terminée.
        """
        result = future.result().result
        status = future.result().status
        
        self.get_logger().info('')
        self.get_logger().info('=================================')
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🎉 NAVIGATION RÉUSSIE!')
            self.get_logger().info(f'   ArUco final: {result.final_aruco_id}')
            self.get_logger().info(f'   Direction: {"AVANT" if result.went_forward else "ARRIÈRE"}')
            self.get_logger().info(f'   Temps: {result.navigation_time:.1f}s')
            self.get_logger().info(f'   Distance: {result.distance_traveled:.2f}m')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('🛑 NAVIGATION ANNULÉE')
            self.get_logger().info(f'   Dernier ArUco: {result.final_aruco_id}')
            self.get_logger().info(f'   Temps: {result.navigation_time:.1f}s')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('❌ NAVIGATION ÉCHOUÉE')
            self.get_logger().info(f'   Dernier ArUco: {result.final_aruco_id}')
            self.get_logger().info(f'   Temps: {result.navigation_time:.1f}s')
        else:
            self.get_logger().error(f'❌ ÉCHEC (status: {status})')
        
        self.get_logger().info('=================================')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Appelé de manière répétée pendant la navigation.
        Affiche une barre de progression!
        """
        feedback = feedback_msg.feedback
        
        # Créer une représentation visuelle
        current_id = feedback.current_aruco_id
        direction_symbol = '➡️' if 'AVANT' in feedback.current_direction else '⬅️'
        
        # Vérifier si un obstacle est détecté
        if feedback.obstacle_detected:
            self.get_logger().warn(
                f'🚨 OBSTACLE! Distance: {feedback.obstacle_distance:.2f}m | '
                f'⏱️  {feedback.elapsed_time:.1f}s | '
                f'{feedback.status_message}'
            )
        else:
            # Afficher la progression normale
            self.get_logger().info(
                f'{direction_symbol} ArUco: {current_id} | '
                f'⏱️  {feedback.elapsed_time:.1f}s | '
                f'{feedback.status_message}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    # Vérifier les arguments de ligne de commande
    if len(sys.argv) != 2:
        print('Usage: ros2 run blue_line_follower aruco_navigation_client <aruco_id>')
        print('Exemple: ros2 run blue_line_follower aruco_navigation_client 5')
        return
    
    try:
        target_id = int(sys.argv[1])
        if target_id < 0 or target_id > 49:
            print('Erreur: Le numéro ArUco doit être entre 0 et 49')
            return
    except ValueError:
        print('Erreur: Veuillez fournir un numéro valide')
        return
    
    # Créer le client
    client = ArucoNavigationClient()
    
    # Envoyer la demande de navigation
    if not client.send_goal(target_id):
        return
    
    # Continuer jusqu'à la fin
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('🛑 Interrompu par l\'utilisateur')
    
    client.destroy_node()


if __name__ == '__main__':
    main()
