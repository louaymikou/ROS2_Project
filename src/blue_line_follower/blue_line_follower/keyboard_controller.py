#!/usr/bin/env python3
"""
Enhanced Keyboard Controller for Line Follower Robot
Controls: Base movement with speed modes
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import time

# Help text
msg = """
╔══════════════════════════════════════════════════════════╗
║     KEYBOARD CONTROLLER - LINE FOLLOWER ROBOT            ║
╠══════════════════════════════════════════════════════════╣
║ MOVEMENT (W/A/S/D):              SPEED MODES:            ║
║   W : Forward                      F1/1 : Slow (0.2 m/s) ║
║   S : Backward                     F2/2 : Medium (0.5)   ║
║   A : Turn Left                    F3/3 : Fast (1.0)     ║
║   D : Turn Right                                         ║
║   SPACE : Stop                                           ║
╠══════════════════════════════════════════════════════════╣
║ QUIT: Ctrl+C                                             ║
╚══════════════════════════════════════════════════════════╝
"""

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # Speed settings
        self.speed_modes = {'slow': 0.2, 'medium': 0.5, 'fast': 1.0}
        self.current_speed = 'medium'
        self.linear_speed = self.speed_modes[self.current_speed]
        self.angular_speed = 1.0
        
        time.sleep(0.5)
        self.get_logger().info('Keyboard Controller Ready!')

    def set_speed_mode(self, mode):
        self.current_speed = mode
        self.linear_speed = self.speed_modes[mode]
        print(f"\n🚀 Speed: {mode.upper()} ({self.linear_speed} m/s)")

    def send_velocity(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = linear * self.linear_speed
        twist.angular.z = angular * self.angular_speed
        for _ in range(3):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    rclpy.init()
    node = KeyboardController()
    
    print(msg)
    print(f"\n✅ Controller Ready! Current speed: {node.current_speed.upper()}\n")
    
    try:
        while True:
            key = get_key().lower()
            
            # Movement
            if key == 'w':
                node.send_velocity(linear=1.0)
                print("▲ Forward")
            elif key == 's':
                node.send_velocity(linear=-1.0)
                print("▼ Backward")
            elif key == 'a':
                node.send_velocity(angular=1.0)
                print("◄ Turn Left")
            elif key == 'd':
                node.send_velocity(angular=-1.0)
                print("► Turn Right")
            elif key == ' ':
                node.send_velocity(0, 0)
                print("■ STOP")
            
            # Speed modes
            elif key == '1':
                node.set_speed_mode('slow')
            elif key == '2':
                node.set_speed_mode('medium')
            elif key == '3':
                node.set_speed_mode('fast')
            
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        print("\n⚠️ Stopping...")
        node.send_velocity(0, 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
