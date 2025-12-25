#!/usr/bin/env python3
"""
Calcul de la distance parcourue par le robot en temps réel
Basé uniquement sur l'odométrie (encodeurs des roues)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class DistanceTracker(Node):
    def __init__(self):
        super().__init__('distance_tracker')
        
        # Subscriber pour l'odométrie
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_cont/odom',
            self.odom_callback,
            10
        )
        
        # Variables pour le calcul de distance
        self.last_position = None
        self.total_distance = 0.0
        self.start_position = None
        self.current_position = None
        
        # Créer un timer pour afficher la distance toutes les secondes
        self.timer = self.create_timer(1.0, self.display_distance)
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║       CALCULATEUR DE DISTANCE PARCOURUE                  ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        self.get_logger().info('📡 En attente des données d\'odométrie...\n')
        
    def odom_callback(self, msg):
        """Callback pour calculer la distance parcourue"""
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Position actuelle
        self.current_position = (current_x, current_y)
        
        # Enregistrer la position de départ
        if self.start_position is None:
            self.start_position = (current_x, current_y)
            self.last_position = (current_x, current_y)
            self.get_logger().info(f'✅ Position de départ enregistrée: ({current_x:.3f}, {current_y:.3f})')
            return
        
        # Calculer la distance depuis la dernière position
        if self.last_position is not None:
            dx = current_x - self.last_position[0]
            dy = current_y - self.last_position[1]
            increment = math.sqrt(dx**2 + dy**2)
            
            # Ajouter à la distance totale (seulement si mouvement significatif)
            if increment > 0.0001:  # Filtre pour éviter le bruit
                self.total_distance += increment
        
        # Mettre à jour la dernière position
        self.last_position = (current_x, current_y)
    
    def get_straight_line_distance(self):
        """Calculer la distance en ligne droite depuis le départ"""
        if self.start_position is None or self.current_position is None:
            return 0.0
        
        dx = self.current_position[0] - self.start_position[0]
        dy = self.current_position[1] - self.start_position[1]
        return math.sqrt(dx**2 + dy**2)
    
    def display_distance(self):
        """Afficher les statistiques de distance"""
        if self.current_position is None:
            return
        
        straight_distance = self.get_straight_line_distance()
        
        # Afficher les informations
        self.get_logger().info('┌──────────────────────────────────────────────────┐')
        self.get_logger().info(f'│ 📏 Distance totale parcourue: {self.total_distance:.3f} m      ')
        self.get_logger().info(f'│ 📐 Distance en ligne droite : {straight_distance:.3f} m      ')
        self.get_logger().info(f'│ 📍 Position actuelle: ({self.current_position[0]:.3f}, {self.current_position[1]:.3f})')
        
        if self.start_position:
            self.get_logger().info(f'│ 🎯 Position départ  : ({self.start_position[0]:.3f}, {self.start_position[1]:.3f})')
        
        # Calculer l'efficacité du trajet
        if self.total_distance > 0.1:
            efficiency = (straight_distance / self.total_distance) * 100
            self.get_logger().info(f'│ 📊 Efficacité trajet: {efficiency:.1f}%           ')
        
        self.get_logger().info('└──────────────────────────────────────────────────┘\n')

def main(args=None):
    rclpy.init(args=args)
    tracker = DistanceTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        print('\n')
        tracker.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        tracker.get_logger().info('║               RÉSUMÉ FINAL                               ║')
        tracker.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
        if tracker.current_position and tracker.start_position:
            straight = tracker.get_straight_line_distance()
            tracker.get_logger().info(f'📏 Distance totale parcourue : {tracker.total_distance:.3f} m')
            tracker.get_logger().info(f'📐 Distance en ligne droite  : {straight:.3f} m')
            tracker.get_logger().info(f'📍 Déplacement (Δx, Δy)      : ({tracker.current_position[0] - tracker.start_position[0]:.3f}, {tracker.current_position[1] - tracker.start_position[1]:.3f}) m')
            
            if tracker.total_distance > 0.1:
                efficiency = (straight / tracker.total_distance) * 100
                tracker.get_logger().info(f'📊 Efficacité du trajet      : {efficiency:.1f}%')
            
            tracker.get_logger().info('\n💡 Distance calculée via odométrie (encodeurs roues)')
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
