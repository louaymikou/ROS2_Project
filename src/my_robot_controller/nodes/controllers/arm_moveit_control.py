#!/usr/bin/env python3
"""
MoveIt Arm Controller - Simple Interface
Control the robot arm using MoveIt2 with predefined poses and custom positions
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose, PoseStamped
import sys


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║           MOVEIT ARM CONTROLLER STARTING                ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
        # Initialize MoveItPy
        try:
            self.moveit = MoveItPy(node=self)
            self.arm = self.moveit.get_planning_component("arm")
            self.gripper = self.moveit.get_planning_component("gripper")
            self.get_logger().info('✅ MoveIt initialized successfully!')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize MoveIt: {e}')
            raise
        
        self.get_logger().info('')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  - home         : Move arm to home position')
        self.get_logger().info('  - extended     : Extend arm forward')
        self.get_logger().info('  - tucked       : Tuck arm compactly')
        self.get_logger().info('  - ready        : Ready for pickup position')
        self.get_logger().info('  - open         : Open gripper')
        self.get_logger().info('  - close        : Close gripper')
        self.get_logger().info('  - half_open    : Half open gripper')
        self.get_logger().info('')
        
    def move_to_named_target(self, group, target_name):
        """Move to a predefined named target"""
        self.get_logger().info(f'📍 Planning to move to: {target_name}')
        
        try:
            # Set target
            if group == "arm":
                self.arm.set_goal_state(configuration_name=target_name)
                plan_result = self.arm.plan()
            else:  # gripper
                self.gripper.set_goal_state(configuration_name=target_name)
                plan_result = self.gripper.plan()
            
            if plan_result:
                self.get_logger().info('✅ Planning successful! Executing...')
                # Execute the plan
                robot_trajectory = plan_result.trajectory
                self.moveit.execute(robot_trajectory, controllers=[])
                self.get_logger().info(f'✅ Successfully moved to {target_name}!')
                return True
            else:
                self.get_logger().error('❌ Planning failed!')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')
            return False
    
    def move_arm_to_joint_values(self, joint_values):
        """
        Move arm to specific joint values
        joint_values: [shoulder, elbow, gripper_rotate]
        """
        self.get_logger().info(f'📍 Moving to joint values: {joint_values}')
        
        try:
            robot_state = RobotState(self.moveit.get_robot_model())
            robot_state.set_joint_group_positions("arm", joint_values)
            
            self.arm.set_goal_state(robot_state=robot_state)
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info('✅ Planning successful! Executing...')
                robot_trajectory = plan_result.trajectory
                self.moveit.execute(robot_trajectory, controllers=[])
                self.get_logger().info('✅ Movement completed!')
                return True
            else:
                self.get_logger().error('❌ Planning failed!')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')
            return False
    
    def move_gripper(self, position):
        """
        Move gripper to specific position
        position: 'open', 'close', or 'half_open'
        """
        return self.move_to_named_target("gripper", position)
    
    def execute_pick_and_place_demo(self):
        """Execute a complete pick and place sequence"""
        self.get_logger().info('🎯 Starting pick and place demo...')
        
        # 1. Go to home position
        self.get_logger().info('Step 1: Moving to home')
        self.move_to_named_target("arm", "home")
        rclpy.spin_once(self, timeout_sec=2.0)
        
        # 2. Open gripper
        self.get_logger().info('Step 2: Opening gripper')
        self.move_gripper("open")
        rclpy.spin_once(self, timeout_sec=2.0)
        
        # 3. Move to ready position
        self.get_logger().info('Step 3: Moving to ready position')
        self.move_to_named_target("arm", "ready")
        rclpy.spin_once(self, timeout_sec=2.0)
        
        # 4. Close gripper (pick)
        self.get_logger().info('Step 4: Closing gripper (picking)')
        self.move_gripper("closed")
        rclpy.spin_once(self, timeout_sec=2.0)
        
        # 5. Lift object
        self.get_logger().info('Step 5: Lifting object')
        self.move_to_named_target("arm", "extended")
        rclpy.spin_once(self, timeout_sec=2.0)
        
        # 6. Open gripper (place)
        self.get_logger().info('Step 6: Opening gripper (placing)')
        self.move_gripper("open")
        rclpy.spin_once(self, timeout_sec=2.0)
        
        # 7. Return to home
        self.get_logger().info('Step 7: Returning to home')
        self.move_to_named_target("arm", "home")
        
        self.get_logger().info('✅ Pick and place demo completed!')


def main(args=None):
    rclpy.init(args=args)
    
    controller = ArmController()
    
    if len(sys.argv) < 2:
        controller.get_logger().info('Usage: ros2 run my_robot_controller arm_moveit_control.py <command>')
        controller.get_logger().info('Commands: home, extended, tucked, ready, open, close, half_open, demo')
        rclpy.shutdown()
        return
    
    command = sys.argv[1].lower()
    
    # Arm commands
    if command in ['home', 'extended', 'tucked', 'ready']:
        controller.move_to_named_target("arm", command)
    
    # Gripper commands
    elif command in ['open', 'close', 'half_open']:
        controller.move_gripper(command)
    
    # Demo
    elif command == 'demo':
        controller.execute_pick_and_place_demo()
    
    else:
        controller.get_logger().error(f'Unknown command: {command}')
        controller.get_logger().info('Valid commands: home, extended, tucked, ready, open, close, half_open, demo')
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
