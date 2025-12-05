import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PS4RobotController(Node):
    def __init__(self):
        super().__init__('ps4_robot_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Subscriber pour la manette
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Mémoire pour les positions
        self.last_shoulder = 0.0
        self.last_elbow = 0.0
        self.last_gripper = 0.0  # Position de la pince
        
        self.get_logger().info('PS4 Controller with Gripper started!')

    def joy_callback(self, msg):
        """
        PS4 Mapping:
        - Joystick gauche: axes[1] (avant/arrière), axes[0] (gauche/droite)
        - Bouton X: buttons[0] = Étendre bras
        - Bouton O: buttons[1] = Ranger bras
        - Bouton Triangle: buttons[2] = Fermer pince
        - Bouton Square: buttons[3] = Ouvrir pince
        - L2/R2: axes[2], axes[5] = Contrôle continu pince
        """
        
        # 1. Contrôle du déplacement
        linear_x = msg.axes[1] * 0.5
        angular_z = msg.axes[0] * 1.0
        
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
        
        # 2. Bouton X = Étendre le bras
        if msg.buttons[0] == 1:
            self.get_logger().info('Bras étendu')
            self.send_arm_command(0.75, 0.75)
        
        # 3. Bouton O = Ranger le bras
        if msg.buttons[1] == 1:
            self.get_logger().info('Bras rangé')
            self.send_arm_command(-1.0, 3.14)

        # 4. Bouton Triangle = Fermer pince
        if msg.buttons[2] == 1:
            self.get_logger().info('Pince fermée')
            self.send_gripper_command(-0.05)  # Positions négatives = fermer
        
        # 5. Bouton Square = Ouvrir pince
        if msg.buttons[3] == 1:
            self.get_logger().info('Pince ouverte')
            self.send_gripper_command(0.0)  # Position 0 = ouvert

    def send_arm_command(self, shoulder_pos, elbow_pos, duration=1.0):
        """Envoie une commande au bras"""
        self.last_shoulder = float(shoulder_pos)
        self.last_elbow = float(elbow_pos)
        
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.last_shoulder, self.last_elbow]
        point.time_from_start.sec = int(duration)
        
        traj.points.append(point)
        self.arm_pub.publish(traj)

    def send_gripper_command(self, gripper_pos, duration=0.5):
        """Envoie une commande à la pince"""
        self.last_gripper = float(gripper_pos)
        
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        # Les deux doigts se déplacent symétriquement
        point.positions = [gripper_pos, gripper_pos]
        point.time_from_start.sec = int(duration)
        
        traj.points.append(point)
        self.gripper_pub.publish(traj)

def main():
    rclpy.init()
    node = PS4RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()