#!/usr/bin/env python3
"""
Interactive Arm Controller
Provides an interactive menu to control the arm with MoveIt2
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import math


class InteractiveArmController(Node):
    def __init__(self):
        super().__init__('interactive_arm_controller')
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║       INTERACTIVE MOVEIT ARM CONTROLLER                 ║')
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
    
    def move_to_named_target(self, group_name, target_name):
        """Move to a predefined named target"""
        print(f'\n📍 Planning to move to: {target_name}')
        
        try:
            if group_name == "arm":
                self.arm.set_goal_state(configuration_name=target_name)
                plan_result = self.arm.plan()
            else:  # gripper
                self.gripper.set_goal_state(configuration_name=target_name)
                plan_result = self.gripper.plan()
            
            if plan_result:
                print('✅ Planning successful! Executing...')
                robot_trajectory = plan_result.trajectory
                self.moveit.execute(robot_trajectory, controllers=[])
                print(f'✅ Successfully moved to {target_name}!')
                return True
            else:
                print('❌ Planning failed!')
                return False
                
        except Exception as e:
            print(f'❌ Error: {e}')
            return False
    
    def move_arm_joints(self, shoulder, elbow, wrist_rotate):
        """Move arm to specific joint angles (in degrees)"""
        # Convert to radians
        joint_values = [
            math.radians(shoulder),
            math.radians(elbow),
            math.radians(wrist_rotate)
        ]
        
        print(f'\n📍 Moving to joint angles: Shoulder={shoulder}°, Elbow={elbow}°, Wrist={wrist_rotate}°')
        
        try:
            robot_state = RobotState(self.moveit.get_robot_model())
            robot_state.set_joint_group_positions("arm", joint_values)
            
            self.arm.set_goal_state(robot_state=robot_state)
            plan_result = self.arm.plan()
            
            if plan_result:
                print('✅ Planning successful! Executing...')
                robot_trajectory = plan_result.trajectory
                self.moveit.execute(robot_trajectory, controllers=[])
                print('✅ Movement completed!')
                return True
            else:
                print('❌ Planning failed!')
                return False
                
        except Exception as e:
            print(f'❌ Error: {e}')
            return False
    
    def show_menu(self):
        """Display interactive menu"""
        print('\n' + '='*60)
        print('           🦾 ARM CONTROL MENU')
        print('='*60)
        print('\n📋 PREDEFINED POSES:')
        print('  1. Home position')
        print('  2. Extended forward')
        print('  3. Tucked (compact)')
        print('  4. Ready for pickup')
        print('\n🤏 GRIPPER CONTROL:')
        print('  5. Open gripper')
        print('  6. Close gripper')
        print('  7. Half open gripper')
        print('\n🎯 CUSTOM CONTROL:')
        print('  8. Set custom joint angles')
        print('  9. Execute pick & place demo')
        print('\n  0. Exit')
        print('='*60)
    
    def run_interactive(self):
        """Run interactive control loop"""
        while rclpy.ok():
            self.show_menu()
            
            try:
                choice = input('\n👉 Enter your choice (0-9): ').strip()
                
                if choice == '0':
                    print('👋 Exiting...')
                    break
                
                elif choice == '1':
                    self.move_to_named_target("arm", "home")
                
                elif choice == '2':
                    self.move_to_named_target("arm", "extended")
                
                elif choice == '3':
                    self.move_to_named_target("arm", "tucked")
                
                elif choice == '4':
                    self.move_to_named_target("arm", "ready")
                
                elif choice == '5':
                    self.move_to_named_target("gripper", "open")
                
                elif choice == '6':
                    self.move_to_named_target("gripper", "closed")
                
                elif choice == '7':
                    self.move_to_named_target("gripper", "half_open")
                
                elif choice == '8':
                    print('\n📐 Enter joint angles in degrees:')
                    shoulder = float(input('  Shoulder (-90 to 90): '))
                    elbow = float(input('  Elbow (-143 to 143): '))
                    wrist = float(input('  Wrist rotation (-180 to 180): '))
                    self.move_arm_joints(shoulder, elbow, wrist)
                
                elif choice == '9':
                    self.execute_demo()
                
                else:
                    print('❌ Invalid choice! Please enter 0-9')
                
                rclpy.spin_once(self, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                print('\n\n👋 Interrupted by user. Exiting...')
                break
            except ValueError:
                print('❌ Invalid input! Please enter a number.')
            except Exception as e:
                print(f'❌ Error: {e}')
    
    def execute_demo(self):
        """Execute pick and place demonstration"""
        print('\n🎯 Starting Pick & Place Demo...')
        print('='*60)
        
        steps = [
            ('Moving to home', lambda: self.move_to_named_target("arm", "home")),
            ('Opening gripper', lambda: self.move_to_named_target("gripper", "open")),
            ('Moving to ready position', lambda: self.move_to_named_target("arm", "ready")),
            ('Closing gripper (pick)', lambda: self.move_to_named_target("gripper", "closed")),
            ('Lifting object', lambda: self.move_to_named_target("arm", "extended")),
            ('Opening gripper (place)', lambda: self.move_to_named_target("gripper", "open")),
            ('Returning home', lambda: self.move_to_named_target("arm", "home")),
        ]
        
        for i, (description, action) in enumerate(steps, 1):
            print(f'\nStep {i}/{len(steps)}: {description}...')
            action()
            rclpy.spin_once(self, timeout_sec=1.0)
        
        print('\n✅ Pick & Place Demo Completed!')
        print('='*60)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = InteractiveArmController()
        controller.run_interactive()
    except Exception as e:
        print(f'❌ Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
