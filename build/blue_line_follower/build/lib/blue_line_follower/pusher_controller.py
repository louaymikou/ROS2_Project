#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class PusherController(Node):
    """
    Contrôleur pour le mécanisme de poussée du robot.
    Permet de pousser des cubes d'arrière vers l'avant du robot.
    """

    def __init__(self):
        super().__init__('pusher_controller')
        
        # Action client pour contrôler le pusher
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/pusher_controller/follow_joint_trajectory'
        )
        
        # Positions prédéfinies (mouvement arrière -> avant)
        self.POSITION_BACK = 0.0        # Position arrière (repos/initial)
        self.POSITION_MIDDLE = 0.3      # Position milieu
        self.POSITION_FRONT = 0.6       # Position avant (complètement étendu)
        
        self.get_logger().info('Pusher Controller initialisé')
        self.get_logger().info('Positions: Arrière=0.0, Milieu=0.3, Avant=0.6')

    def wait_for_server(self, timeout_sec=10.0):
        """Attend que le serveur d'action soit disponible"""
        self.get_logger().info('En attente du serveur d\'action pusher_controller...')
        if not self._action_client.wait_for_server(timeout_sec):
            self.get_logger().error('Le serveur pusher_controller n\'est pas disponible!')
            return False
        self.get_logger().info('Serveur pusher_controller connecté!')
        return True

    def move_pusher(self, position, duration_sec=2.0):
        """
        Déplace le pousseur à une position spécifique.
        
        Args:
            position: Position cible (0.0 à 0.6)
            duration_sec: Durée du mouvement en secondes
        """
        # Créer le goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['pusher_joint']
        
        # Point de trajectoire
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'Envoi du pousseur à la position {position}...')
        
        # Envoyer le goal
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejeté par le serveur')
            return False
        
        self.get_logger().info('Goal accepté, en attente de la fin du mouvement...')
        
        # Attendre la fin du mouvement
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'✓ Pousseur arrivé à la position {position}')
            return True
        else:
            self.get_logger().error(f'Échec du mouvement (status: {result.status})')
            return False

    def push_forward(self, duration_sec=3.0):
        """Pousse vers l'avant"""
        self.get_logger().info('➡️  POUSSÉE VERS L\'AVANT')
        return self.move_pusher(self.POSITION_FRONT, duration_sec)

    def push_middle(self, duration_sec=2.0):
        """Pousse à mi-chemin"""
        self.get_logger().info('⏸️  POSITION MILIEU')
        return self.move_pusher(self.POSITION_MIDDLE, duration_sec)

    def return_to_back(self, duration_sec=2.0):
        """Retourne à la position arrière"""
        self.get_logger().info('⬅️  RETOUR À L\'ARRIÈRE')
        return self.move_pusher(self.POSITION_BACK, duration_sec)

    def push_sequence_forward(self):
        """
        Séquence complète de poussée vers l'avant:
        1. Pousser vers l'avant
        2. Pause
        3. Retour à l'arrière
        """
        self.get_logger().info('=== Démarrage séquence de poussée AVANT ===')
        
        if not self.push_forward(3.0):
            return False
        
        time.sleep(1.0)  # Pause pour laisser le cube tomber/glisser
        
        if not self.return_to_back(3.0):
            return False
        
        self.get_logger().info('=== Séquence AVANT terminée ===')
        return True

    def push_sequence_partial(self):
        """
        Séquence de poussée partielle (milieu):
        1. Pousser jusqu'au milieu
        2. Pause
        3. Retour à l'arrière
        """
        self.get_logger().info('=== Démarrage séquence PARTIELLE ===')
        
        if not self.push_middle(2.0):
            return False
        
        time.sleep(0.5)
        
        if not self.return_to_back(2.0):
            return False
        
        self.get_logger().info('=== Séquence PARTIELLE terminée ===')
        return True


def main(args=None):
    rclpy.init(args=args)
    
    pusher = PusherController()
    
    if not pusher.wait_for_server():
        pusher.destroy_node()
        rclpy.shutdown()
        return
    
    try:
        # Menu interactif
        while rclpy.ok():
            print("\n" + "="*50)
            print("🤖 CONTRÔLEUR DE POUSSEUR - Menu")
            print("="*50)
            print("1. Pousser vers l'AVANT")
            print("2. Position MILIEU")
            print("3. Retour à l'ARRIÈRE")
            print("4. Séquence complète AVANT")
            print("5. Séquence PARTIELLE (milieu)")
            print("6. Position personnalisée")
            print("0. Quitter")
            print("="*50)
            
            choice = input("Votre choix: ")
            
            if choice == '1':
                pusher.push_forward()
            elif choice == '2':
                pusher.push_middle()
            elif choice == '3':
                pusher.return_to_back()
            elif choice == '4':
                pusher.push_sequence_forward()
            elif choice == '5':
                pusher.push_sequence_partial()
            elif choice == '6':
                try:
                    pos = float(input("Position (0.0 à 0.6): "))
                    if 0.0 <= pos <= 0.6:
                        pusher.move_pusher(pos)
                    else:
                        print("⚠️ Position hors limites!")
                except ValueError:
                    print("⚠️ Valeur invalide!")
            elif choice == '0':
                print("Au revoir!")
                break
            else:
                print("⚠️ Choix invalide!")
    
    except KeyboardInterrupt:
        pusher.get_logger().info('Arrêt demandé par l\'utilisateur')
    
    finally:
        pusher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
