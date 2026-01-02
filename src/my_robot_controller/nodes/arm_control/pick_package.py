#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class PickPackage(Node):
    def __init__(self):
        super().__init__('pick_package')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
    def move_arm(self, shoulder, elbow, gripper_rotate, duration_sec=3):
        self.arm_client.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [shoulder, elbow, gripper_rotate]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)  # ✅ int()
        goal.trajectory.points = [point]
        
        self.get_logger().info(f'Déplacement bras: shoulder={shoulder:.2f}, elbow={elbow:.2f}')
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
    def move_gripper(self, position, duration_sec=2):
        self.gripper_client.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position, -position]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)  # ✅ int()
        goal.trajectory.points = [point]
        
        action = "Ouverture" if position > 0 else "Fermeture"
        self.get_logger().info(f'{action} gripper: {position}')
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
    def pick_sequence(self):
        self.get_logger().info('🦾 SÉQUENCE DE RAMASSAGE')
        
        # 1. Ouvrir le gripper
        self.get_logger().info('1️⃣ Ouverture gripper')
        self.move_gripper(0.02, 2)
        
        # 2. Tendre le bras vers l'avant et descendre
        self.get_logger().info('2️⃣ Extension bras vers package')
        self.move_arm(shoulder=1.2, elbow=-1.5, gripper_rotate=0.0, duration_sec=4)
        
        # 3. Attendre stabilisation
        import time
        time.sleep(1)
        
        # 4. Fermer le gripper pour attraper
        self.get_logger().info('3️⃣ Fermeture gripper - ATTRAPER')
        self.move_gripper(-0.01, 2)
        
        # 5. Attendre que le gripper se ferme
        time.sleep(1)
        
        # 6. Ramener le bras
        self.get_logger().info('4️⃣ Ramener le package')
        self.move_arm(shoulder=0.5, elbow=-0.5, gripper_rotate=0.0, duration_sec=3)
        
        self.get_logger().info('✅ RAMASSAGE TERMINÉ !')

def main():
    rclpy.init()
    node = PickPackage()
    node.pick_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
