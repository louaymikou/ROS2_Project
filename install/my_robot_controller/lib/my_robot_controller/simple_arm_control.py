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
   q : Quitter
"""

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # MEMOIRE : On retient la dernière position connue pour ne pas tout réinitialiser
        self.last_shoulder = 0.0
        self.last_elbow = 0.0
        self.last_rotate = 0.0  # AJOUTER LA MÉMOIRE DE ROTATION

    def send_command(self, shoulder_pos, elbow_pos, rotate_pos, duration=1.0):
        # 1. Mise à jour de la mémoire
        self.last_shoulder = float(shoulder_pos)
        self.last_elbow = float(elbow_pos)
        self.last_rotate = float(rotate_pos) # Mettre à jour la rotation

        # 2. Création du message
        traj = JointTrajectory()
        # Mettre à jour les noms des articulations
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        # Envoyer les 3 positions
        point.positions = [self.last_shoulder, self.last_elbow, self.last_rotate]
        point.time_from_start.sec = int(duration)
        
        traj.points.append(point)
        self.publisher_.publish(traj)
        print(f"Commande envoyée -> Epaule: {self.last_shoulder:.2f}, Coude: {self.last_elbow:.2f}, Rotation: {self.last_rotate:.2f}")

    def move_single_motor(self, motor_choice, angle):
        """
        Bouge un seul moteur en gardant les autres fixes
        """
        if motor_choice == 1: # Epaule
            print(f"Déplacement de l'épaule vers {angle}")
            self.send_command(angle, self.last_elbow, self.last_rotate)
        
        elif motor_choice == 2: # Coude
            print(f"Déplacement du coude vers {angle}")
            self.send_command(self.last_shoulder, angle, self.last_rotate)
        
        elif motor_choice == 3: # Rotation Pince
            print(f"Rotation de la pince vers {angle}")
            self.send_command(self.last_shoulder, self.last_elbow, angle)

        else:
            print("Erreur : Moteur inconnu (choisir 1, 2 ou 3)")

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
                # Conserver la rotation actuelle en étendant
                node.send_command(0.75, 0.75, node.last_rotate)
            
            elif key == 'r':
                # Conserver la rotation actuelle en rangeant
                node.send_command(-1.0, 3.14, node.last_rotate)
            
            elif key == 'q':
                print("Fermeture...")
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
                
                print("Retour au mode clavier (appuyez sur e, r, m, q)...")
                # On remet le mode raw pour la prochaine boucle
                # (Pas besoin de code ici, la fonction get_key() le fera au début de la boucle)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()