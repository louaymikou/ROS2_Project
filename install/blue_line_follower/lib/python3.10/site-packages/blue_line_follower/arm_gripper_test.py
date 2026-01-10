#!/usr/bin/env python3
"""
Script de test pour le bras et le gripper
Lance une séquence de mouvements prédéfinis
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class ArmGripperTest(Node):
    def __init__(self):
        super().__init__('arm_gripper_test')
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Attendre que les contrôleurs soient prêts
        time.sleep(2.0)
        
        self.get_logger().info('🤖 Début du test du bras et gripper...')
        
    def send_arm_command(self, shoulder, elbow, rotate, duration=2.0):
        """Envoi une commande au bras"""
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(shoulder), float(elbow), float(rotate)]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        traj.points.append(point)
        self.arm_publisher.publish(traj)
        self.get_logger().info(f'📍 Bras -> Epaule: {shoulder:.2f}, Coude: {elbow:.2f}, Rotation: {rotate:.2f}')
        
    def send_gripper_command(self, left, right, duration=1.0):
        """Envoi une commande au gripper"""
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(left), float(right)]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        traj.points.append(point)
        self.gripper_publisher.publish(traj)
        self.get_logger().info(f'✋ Gripper -> Gauche: {left:.2f}, Droite: {right:.2f}')
        
    def run_test_sequence(self):
        """Execute une séquence de test"""
        
        # 1. Position initiale
        self.get_logger().info('\n📌 Test 1: Position initiale (repos)')
        self.send_arm_command(0.0, 0.0, 0.0, duration=2.0)
        time.sleep(3.0)
        
        # 2. Ouvrir le gripper
        self.get_logger().info('\n📌 Test 2: Ouvrir le gripper')
        self.send_gripper_command(0.0, 0.0, duration=1.0)
        time.sleep(2.0)
        
        # 3. Étendre le bras (position travail)
        self.get_logger().info('\n📌 Test 3: Étendre le bras')
        self.send_arm_command(0.75, 0.75, 0.0, duration=3.0)
        time.sleep(4.0)
        
        # 4. Rotation de la pince à 90°
        self.get_logger().info('\n📌 Test 4: Rotation pince 90°')
        self.send_arm_command(0.75, 0.75, 1.57, duration=2.0)
        time.sleep(3.0)
        
        # 5. Fermer le gripper
        self.get_logger().info('\n📌 Test 5: Fermer le gripper')
        self.send_gripper_command(-0.5, -0.5, duration=1.0)
        time.sleep(2.0)
        
        # 6. Rotation de la pince à -90°
        self.get_logger().info('\n📌 Test 6: Rotation pince -90°')
        self.send_arm_command(0.75, 0.75, -1.57, duration=2.0)
        time.sleep(3.0)
        
        # 7. Ouvrir le gripper
        self.get_logger().info('\n📌 Test 7: Ouvrir le gripper')
        self.send_gripper_command(0.0, 0.0, duration=1.0)
        time.sleep(2.0)
        
        # 8. Ranger le bras (position repliée)
        self.get_logger().info('\n📌 Test 8: Ranger le bras')
        self.send_arm_command(-1.0, 3.14, 0.0, duration=3.0)
        time.sleep(4.0)
        
        # 9. Retour à la position initiale
        self.get_logger().info('\n📌 Test 9: Retour position initiale')
        self.send_arm_command(0.0, 0.0, 0.0, duration=3.0)
        time.sleep(4.0)
        
        self.get_logger().info('\n✅ Test terminé avec succès!')


def main():
    rclpy.init()
    
    node = ArmGripperTest()
    
    try:
        # Lancer la séquence de test
        node.run_test_sequence()
        
    except KeyboardInterrupt:
        node.get_logger().info('Test interrompu par l\'utilisateur')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
