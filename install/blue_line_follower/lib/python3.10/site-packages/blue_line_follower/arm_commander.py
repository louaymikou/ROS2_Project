#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

# Texte d'aide
msg = """
CONTROLE DU BRAS AMELIORE
---------------------------
Touches :
   e : Etendre (Position travail)
   r : Ranger (Position repliée)
   m : Mode MANUEL (Choisir moteur et angle)
   g : Gripper OUVERT
   c : Gripper FERME
   q : Quitter
"""

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # MEMOIRE : On retient la dernière position connue pour ne pas tout réinitialiser
        self.last_shoulder = 0.0
        self.last_elbow = 0.0
        self.last_rotate = 0.0
        self.last_gripper_left = 0.0
        self.last_gripper_right = 0.0

    def send_arm_command(self, shoulder_pos, elbow_pos, rotate_pos, duration=1.0):
        """Envoi commande pour le bras (épaule, coude, rotation)"""
        # 1. Mise à jour de la mémoire
        self.last_shoulder = float(shoulder_pos)
        self.last_elbow = float(elbow_pos)
        self.last_rotate = float(rotate_pos)

        # 2. Création du message
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.last_shoulder, self.last_elbow, self.last_rotate]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        traj.points.append(point)
        self.arm_publisher.publish(traj)
        self.get_logger().info(f'Commande bras -> Epaule: {self.last_shoulder:.2f}, Coude: {self.last_elbow:.2f}, Rotation: {self.last_rotate:.2f}')

    def send_gripper_command(self, left_pos, right_pos, duration=1.0):
        """Envoi commande pour le gripper"""
        # 1. Mise à jour de la mémoire
        self.last_gripper_left = float(left_pos)
        self.last_gripper_right = float(right_pos)

        # 2. Création du message
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.last_gripper_left, self.last_gripper_right]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        traj.points.append(point)
        self.gripper_publisher.publish(traj)
        self.get_logger().info(f'Commande gripper -> Gauche: {self.last_gripper_left:.2f}, Droite: {self.last_gripper_right:.2f}')

    def move_single_motor(self, motor_choice, angle):
        """Bouge un seul moteur en gardant les autres fixes"""
        if motor_choice == 1:  # Epaule
            self.get_logger().info(f'Déplacement de l\'épaule vers {angle}')
            self.send_arm_command(angle, self.last_elbow, self.last_rotate)
        
        elif motor_choice == 2:  # Coude
            self.get_logger().info(f'Déplacement du coude vers {angle}')
            self.send_arm_command(self.last_shoulder, angle, self.last_rotate)
        
        elif motor_choice == 3:  # Rotation Pince
            self.get_logger().info(f'Rotation de la pince vers {angle}')
            self.send_arm_command(self.last_shoulder, self.last_elbow, angle)

        else:
            self.get_logger().error('Erreur : Moteur inconnu (choisir 1, 2 ou 3)')

# --- Gestion du terminal (Ne pas toucher) ---
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
    node = ArmCommander()
    
    print(msg)
    
    try:
        while True:
            key = get_key()
            
            if key == 'e':
                # Etendre le bras (Position travail)
                node.send_arm_command(0.75, 0.75, node.last_rotate)
            
            elif key == 'r':
                # Ranger le bras (Position repliée)
                node.send_arm_command(-1.0, 3.14, node.last_rotate)
            
            elif key == 'g':
                # Ouvrir le gripper
                node.send_gripper_command(0.0, 0.0)
            
            elif key == 'c':
                # Fermer le gripper
                node.send_gripper_command(-0.5, -0.5)
            
            elif key == 'q':
                print("\nFermeture...")
                break
            
            elif key == 'm':
                # On restaure le terminal normal pour pouvoir écrire du texte
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, termios.tcgetattr(sys.stdout))
                
                print("\n--- MODE MANUEL ---")
                try:
                    mot = int(input("Quel moteur ? (1=Epaule, 2=Coude, 3=Rotation) : "))
                    ang = float(input("Quel angle ? (ex: 1.57 pour 90°) : "))
                    node.move_single_motor(mot, ang)
                except ValueError:
                    print("Erreur : Entrez des nombres valides !")
                
                print("Retour au mode clavier (appuyez sur e, r, m, g, c, q)...")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
