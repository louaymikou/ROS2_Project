#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SafeForward(Node):
    def __init__(self):
        super().__init__('safe_forward')
        self.pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.safe_distance = 1.0  # ✅ Augmenté de 0.5 à 1.0m
        
    def scan_cb(self, msg):
        # Regarder devant (90 degrés devant)
        front_ranges = msg.ranges[len(msg.ranges)//3 : 2*len(msg.ranges)//3]
        front_ranges = [r for r in front_ranges if 0.1 < r < 10.0]
        
        cmd = Twist()
        if front_ranges and min(front_ranges) > self.safe_distance:
            cmd.linear.x = 0.15  # ✅ Vitesse réduite de 0.2 à 0.15 m/s
            self.get_logger().info(f'✅ Avance - Distance: {min(front_ranges):.2f}m')
        else:
            cmd.linear.x = 0.0  # Arrêter
            if front_ranges:
                self.get_logger().warn(f'🛑 STOP ! Obstacle à {min(front_ranges):.2f}m')
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = SafeForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Arrêt d'urgence
        cmd = Twist()
        node.pub.publish(cmd)

if __name__ == '__main__':
    main()
