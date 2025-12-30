#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class Backup3m(Node):
    def __init__(self):
        super().__init__('backup_3m')
        self.pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_cb, 10)
        self.start_x = None
        self.start_y = None
        self.target_distance = 3.0
        self.speed = -0.2  # Vitesse de recul
        
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(f'🚗 Départ: ({x:.2f}, {y:.2f})')
        
        distance = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
        
        cmd = Twist()
        if distance < self.target_distance:
            cmd.linear.x = self.speed
            self.pub.publish(cmd)
            self.get_logger().info(f'Recul... {distance:.2f}m / {self.target_distance}m')
        else:
            cmd.linear.x = 0.0
            self.pub.publish(cmd)
            self.get_logger().info(f'✅ STOP ! Distance parcourue: {distance:.2f}m')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = Backup3m()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cmd = Twist()
        node.pub.publish(cmd)

if __name__ == '__main__':
    main()
