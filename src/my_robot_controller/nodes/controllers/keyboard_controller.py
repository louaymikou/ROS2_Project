#!/usr/bin/env python3
"""
Enhanced Keyboard Controller for ROS2 Mobile Manipulator
Controls: Base movement, Arm (incremental), Gripper, with speed modes
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty
import time

# Help text
msg = """
╔══════════════════════════════════════════════════════════╗
║     ENHANCED ROBOT KEYBOARD CONTROLLER                   ║
╠══════════════════════════════════════════════════════════╣
║ MOVEMENT:                        SPEED MODES:            ║
║   I : Forward                      1 : Slow (0.2 m/s)    ║
║   K : Backward                     2 : Medium (0.5)      ║
║   J : Turn Left                    3 : Fast (1.0)        ║
║   L : Turn Right                                         ║
║   SPACE : Stop                                           ║
╠══════════════════════════════════════════════════════════╣
║ ARM CONTROL (Incremental):       GRIPPER:                ║
║   A/Q : Shoulder Up/Down           O : Open Gripper      ║
║   Z/S : Elbow Out/In               P : Close Gripper     ║
║   R/F : Rotate Left/Right          0 : Home Position     ║
╠══════════════════════════════════════════════════════════╣
║ QUIT: W or Ctrl+C                                        ║
╚══════════════════════════════════════════════════════════╝
"""

class EnhancedKeyboardController(Node):
    def __init__(self):
        super().__init__('enhanced_keyboard_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Speed settings
        self.speed_modes = {'slow': 0.2, 'medium': 0.5, 'fast': 1.0}
        self.current_speed = 'medium'
        self.linear_speed = self.speed_modes[self.current_speed]
        self.angular_speed = 1.0
        
        # Arm joint positions (incremental control)
        self.shoulder_pos = 0.0
        self.elbow_pos = 0.0
        self.gripper_rotate_pos = 0.0
        
        # Arm joint limits
        self.shoulder_limits = (-1.57, 1.57)
        self.elbow_limits = (-2.5, 2.5)
        self.gripper_rot_limits = (-3.14, 3.14)
        
        # Increment step for arm
        self.arm_step = 0.15
        
        time.sleep(0.5)
        self.get_logger().info('Enhanced Keyboard Controller Ready!')

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

    def send_arm_command(self, duration=0.3):
        # Clamp values to limits
        self.shoulder_pos = max(self.shoulder_limits[0], min(self.shoulder_limits[1], self.shoulder_pos))
        self.elbow_pos = max(self.elbow_limits[0], min(self.elbow_limits[1], self.elbow_pos))
        self.gripper_rotate_pos = max(self.gripper_rot_limits[0], min(self.gripper_rot_limits[1], self.gripper_rotate_pos))
        
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.shoulder_pos, self.elbow_pos, self.gripper_rotate_pos]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(duration * 1e9)
        
        traj.points.append(point)
        self.arm_pub.publish(traj)
        
        print(f"  Arm: shoulder={self.shoulder_pos:.2f} elbow={self.elbow_pos:.2f} rotate={self.gripper_rotate_pos:.2f}")

    def send_gripper_command(self, opening):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [-opening, -opening]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.5 * 1e9)
        
        traj.points.append(point)
        self.gripper_pub.publish(traj)

    def home_arm(self):
        self.shoulder_pos = 0.0
        self.elbow_pos = 0.0
        self.gripper_rotate_pos = 0.0
        self.send_arm_command()
        print("🏠 Arm: Home position")


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
    node = EnhancedKeyboardController()
    
    print(msg)
    print(f"\n✅ Controller Ready! Current speed: {node.current_speed.upper()}\n")
    
    try:
        while True:
            key = get_key().lower()
            
            # Movement
            if key == 'i':
                node.send_velocity(linear=1.0)
                print("▲ Forward")
            elif key == 'k':
                node.send_velocity(linear=-1.0)
                print("▼ Backward")
            elif key == 'j':
                node.send_velocity(angular=1.0)
                print("◄ Turn Left")
            elif key == 'l':
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
            
            # Arm control (incremental)
            elif key == 'a':
                node.shoulder_pos += node.arm_step
                node.send_arm_command()
            elif key == 'q':
                node.shoulder_pos -= node.arm_step
                node.send_arm_command()
            elif key == 'z':
                node.elbow_pos -= node.arm_step
                node.send_arm_command()
            elif key == 's':
                node.elbow_pos += node.arm_step
                node.send_arm_command()
            elif key == 'r':
                node.gripper_rotate_pos += node.arm_step
                node.send_arm_command()
            elif key == 'f':
                node.gripper_rotate_pos -= node.arm_step
                node.send_arm_command()
            
            # Gripper
            elif key == 'o':
                node.send_gripper_command(0.15)
                print("🤏 Gripper: OPEN")
            elif key == 'p':
                node.send_gripper_command(0.0)
                print("✊ Gripper: CLOSED")
            
            # Home position
            elif key == '0':
                node.home_arm()
            
            # Quit
            elif key == 'w':
                print("\n👋 Quitting...")
                break
            
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        print("\n⚠️ Stopping...")
        node.send_velocity(0, 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()