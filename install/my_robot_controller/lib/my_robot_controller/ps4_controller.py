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
        
        # Subscriber pour la manette
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Mémoire pour les positions du bras
        self.last_shoulder = 0.0
        self.last_elbow = 0.0
        
        self.get_logger().info('PS4 Controller started!')

    def joy_callback(self, msg):
        """
        Callback appelé à chaque fois qu'une entrée de la manette est reçue
        
        PS4 Mapping:
        - Joystick gauche: axes[1] (avant/arrière), axes[0] (gauche/droite)
        - Bouton X: buttons[0]
        - Bouton O: buttons[1]
        """
        
        # 1. Contrôle du déplacement avec le joystick gauche
        linear_x = msg.axes[1] * 0.5  # Avant/Arrière (axe Y)
        angular_z = msg.axes[0] * 1.0  # Rotation (axe X)
        
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
        
        # 2. Bouton X (buttons[0]) = Étendre le bras
        if msg.buttons[0] == 1:
            self.get_logger().info('Bouton X pressé: Étendre le bras')
            self.send_arm_command(0.75, 0.75)  # Positions d'extension
        
        # 3. Bouton O (buttons[1]) = Ranger le bras
        if msg.buttons[1] == 1:
            self.get_logger().info('Bouton O pressé: Ranger le bras')
            self.send_arm_command(-1.0, 3.14)  # Positions repliées

    def send_arm_command(self, shoulder_pos, elbow_pos, duration=1.0):
        """Envoie une commande de trajectoire au bras"""
        self.last_shoulder = float(shoulder_pos)
        self.last_elbow = float(elbow_pos)
        
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.last_shoulder, self.last_elbow]
        point.time_from_start.sec = int(duration)
        
        traj.points.append(point)
        self.arm_pub.publish(traj)

def main():
    rclpy.init()
    node = PS4RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()