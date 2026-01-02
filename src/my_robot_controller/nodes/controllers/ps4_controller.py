import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class PS4RobotController(Node):
    def __init__(self):
        super().__init__('ps4_robot_controller')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Mémoriser la dernière position du bras pour la rotation
        self.shoulder_pos = 0.0
        self.elbow_pos = 0.0
        self.gripper_rotate_pos = 0.0
        
        # Mémoriser l'état précédent des boutons pour ne déclencher qu'une fois
        self.prev_buttons = None
        
        self.get_logger().info("PS4 Controller corrigé démarré!")

    def joy_callback(self, msg):
        # Initialiser l'état des boutons au premier message
        if self.prev_buttons is None:
            self.prev_buttons = [0] * len(msg.buttons)

        # --- Commande Mouvement Base (Continu) ---
        twist = Twist()
        twist.linear.x = msg.axes[1] * 0.5  # Stick Gauche Y (Avancer/Reculer)
        twist.angular.z = msg.axes[0] * 0.5  # Stick Gauche X (Pivoter)
        self.cmd_vel_pub.publish(twist)

        # --- Commandes du Bras (Déclenché une seule fois par appui) ---
        # Carré (Square) -> Position étendue
        if msg.buttons[3] and not self.prev_buttons[3]:
            self.get_logger().info("Bras: Position étendue")
            self.shoulder_pos = 0.75
            self.elbow_pos = 0.75
            self.send_arm_command(self.shoulder_pos, self.elbow_pos, self.gripper_rotate_pos)

        # Croix (X) -> Position rangée
        if msg.buttons[0] and not self.prev_buttons[0]:
            self.get_logger().info("Bras: Position rangée")
            self.shoulder_pos = -1.0
            self.elbow_pos = 3.14
            self.send_arm_command(self.shoulder_pos, self.elbow_pos, self.gripper_rotate_pos)

        # --- Commandes de la Pince (Déclenché une seule fois par appui) ---
        # Triangle -> Fermer la pince
        if msg.buttons[2] and not self.prev_buttons[2]:
            self.get_logger().info("Pince: Fermeture")
            self.send_gripper_command(0.0)

        # Rond (Circle) -> Ouvrir la pince
        if msg.buttons[1] and not self.prev_buttons[1]:
            self.get_logger().info("Pince: Ouverture")
            self.send_gripper_command(0.12)

        # --- Rotation de la Pince (Modifié pour un seul déclenchement) ---
        # L1 -> Rotation anti-horaire
        if msg.buttons[4] and not self.prev_buttons[4]:
            self.get_logger().info("Pince: Rotation anti-horaire")
            self.gripper_rotate_pos = min(self.gripper_rotate_pos + 0.2, 3.14159) # Incrément plus grand
            self.send_arm_command(self.shoulder_pos, self.elbow_pos, self.gripper_rotate_pos)

        # R1 -> Rotation horaire
        if msg.buttons[5] and not self.prev_buttons[5]:
            self.get_logger().info("Pince: Rotation horaire")
            self.gripper_rotate_pos = max(self.gripper_rotate_pos - 0.2, -3.14159) # Incrément plus grand
            self.send_arm_command(self.shoulder_pos, self.elbow_pos, self.gripper_rotate_pos)
            
        # Mettre à jour l'état des boutons pour le prochain cycle
        self.prev_buttons = msg.buttons

    def send_arm_command(self, shoulder_pos, elbow_pos, rotate_pos, duration=0.5):
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [shoulder_pos, elbow_pos, rotate_pos]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        traj.points.append(point)
        self.arm_pub.publish(traj)

    def send_gripper_command(self, opening_distance, duration=0.5):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [-opening_distance, -opening_distance]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        traj.points.append(point)
        self.gripper_pub.publish(traj)


def main():
    rclpy.init()
    controller = PS4RobotController()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()