#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class BackwardToWall(Node):
    def __init__(self):
        super().__init__('backward_to_wall')
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.min_distance = 0.1  # Arrêt à 0.1m
        self.backward_speed = -0.1  # Vitesse de recul
        
    def scan_callback(self, msg):
        # Regarder derrière (180°) - index milieu des lectures arrières
        rear_ranges = msg.ranges[len(msg.ranges)//2 - 50 : len(msg.ranges)//2 + 50]
        rear_ranges = [r for r in rear_ranges if r > 0.0]  # Filtrer les valeurs invalides
        
        if rear_ranges:
            min_rear_distance = min(rear_ranges)
            
            if min_rear_distance > self.min_distance:
                # Reculer
                cmd = Twist()
                cmd.linear.x = self.backward_speed
                self.publisher.publish(cmd)
                self.get_logger().info(f'Recul... Distance arrière: {min_rear_distance:.2f}m')
            else:
                # Arrêter
                cmd = Twist()
                cmd.linear.x = 0.0
                self.publisher.publish(cmd)
                self.get_logger().info(f'✅ ARRÊT ! Distance mur: {min_rear_distance:.2f}m')
                rclpy.shutdown()

def main():
    rclpy.init()
    node = BackwardToWall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Arrêt final
        cmd = Twist()
        node.publisher.publish(cmd)
        node.destroy_node()

if __name__ == '__main__':
    main()
