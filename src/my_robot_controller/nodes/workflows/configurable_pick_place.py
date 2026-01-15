#!/usr/bin/env python3
"""
Configurable Pick and Place Workflow
Allows customization of waypoints via parameters or command line
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from moveit.planning import MoveItPy
from action_msgs.msg import GoalStatus
import sys
import json
import math
import time


class ConfigurablePickPlace(Node):
    def __init__(self, config_file=None):
        super().__init__('configurable_pick_place')
        
        # Load configuration
        self.config = self.load_config(config_file)
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║     CONFIGURABLE PICK AND PLACE WORKFLOW                ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
        # Navigation
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        
        # MoveIt
        self.get_logger().info('🤖 Initializing MoveIt...')
        try:
            self.moveit = MoveItPy(node=self)
            self.arm = self.moveit.get_planning_component("arm")
            self.gripper = self.moveit.get_planning_component("gripper")
            self.get_logger().info('✅ MoveIt ready!')
        except Exception as e:
            self.get_logger().error(f'❌ MoveIt init failed: {e}')
            raise
        
        # Wait for Nav2
        self.get_logger().info('⏳ Waiting for Nav2...')
        self._nav_action_client.wait_for_server()
        self.get_logger().info('✅ Nav2 ready!')
        
        self.print_configuration()
    
    def load_config(self, config_file):
        """Load configuration from file or use defaults"""
        default_config = {
            'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'pick_location': {'x': 2.0, 'y': 1.0, 'yaw': 0.0},
            'place_location': {'x': -2.0, 'y': -1.0, 'yaw': 1.57},
            'pick_arm_sequence': ['ready', 'extended', 'closed', 'ready', 'tucked'],
            'place_arm_sequence': ['ready', 'extended', 'open', 'ready', 'home'],
            'delays': {
                'between_steps': 2.0,
                'arm_movement': 1.0,
                'gripper_action': 1.0
            }
        }
        
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                    self.get_logger().info(f'✅ Loaded config from: {config_file}')
            except Exception as e:
                self.get_logger().warn(f'⚠️  Could not load config file: {e}')
                self.get_logger().info('Using default configuration')
        
        return default_config
    
    def print_configuration(self):
        """Print current configuration"""
        self.get_logger().info('')
        self.get_logger().info('📋 Current Configuration:')
        self.get_logger().info(f'   Home: ({self.config["home"]["x"]:.2f}, {self.config["home"]["y"]:.2f})')
        self.get_logger().info(f'   Pick: ({self.config["pick_location"]["x"]:.2f}, {self.config["pick_location"]["y"]:.2f})')
        self.get_logger().info(f'   Place: ({self.config["place_location"]["x"]:.2f}, {self.config["place_location"]["y"]:.2f})')
        self.get_logger().info('')
    
    def navigate_to(self, location_name):
        """Navigate to a configured location"""
        loc = self.config.get(location_name, self.config['home'])
        x, y, yaw = loc['x'], loc['y'], loc['yaw']
        
        self.get_logger().info(f'🧭 Navigating to {location_name}: ({x:.2f}, {y:.2f})')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED
    
    def execute_arm_action(self, action):
        """Execute arm or gripper action"""
        if action in ['open', 'closed', 'half_open']:
            # Gripper action
            component = self.gripper
            action_type = 'Gripper'
        else:
            # Arm action
            component = self.arm
            action_type = 'Arm'
        
        self.get_logger().info(f'🦾 {action_type}: {action}')
        
        try:
            component.set_goal_state(configuration_name=action)
            plan = component.plan()
            if plan:
                self.moveit.execute(plan.trajectory, controllers=[])
                time.sleep(self.config['delays']['arm_movement'])
                return True
        except Exception as e:
            self.get_logger().error(f'❌ {action_type} action failed: {e}')
        
        return False
    
    def execute_workflow(self):
        """Execute the complete workflow"""
        self.get_logger().info('')
        self.get_logger().info('╔═══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║          STARTING PICK AND PLACE WORKFLOW                ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════════════╝')
        
        try:
            # Step 1: Go to home
            self.get_logger().info('\n📍 STEP 1: Initialize at home')
            if not self.execute_arm_action('home'):
                return False
            time.sleep(self.config['delays']['between_steps'])
            
            # Step 2: Navigate to pick location
            self.get_logger().info('\n📍 STEP 2: Navigate to pick location')
            if not self.navigate_to('pick_location'):
                self.get_logger().error('❌ Navigation to pick location failed')
                return False
            time.sleep(self.config['delays']['between_steps'])
            
            # Step 3: Pick sequence
            self.get_logger().info('\n📍 STEP 3: Execute pick sequence')
            self.execute_arm_action('open')
            for action in self.config['pick_arm_sequence']:
                if not self.execute_arm_action(action):
                    return False
            time.sleep(self.config['delays']['between_steps'])
            
            # Step 4: Navigate to place location
            self.get_logger().info('\n📍 STEP 4: Navigate to place location')
            if not self.navigate_to('place_location'):
                self.get_logger().error('❌ Navigation to place location failed')
                return False
            time.sleep(self.config['delays']['between_steps'])
            
            # Step 5: Place sequence
            self.get_logger().info('\n📍 STEP 5: Execute place sequence')
            for action in self.config['place_arm_sequence']:
                if not self.execute_arm_action(action):
                    return False
            time.sleep(self.config['delays']['between_steps'])
            
            # Step 6: Return home
            self.get_logger().info('\n📍 STEP 6: Return to home position')
            if not self.navigate_to('home'):
                self.get_logger().error('❌ Navigation to home failed')
                return False
            
            self.execute_arm_action('home')
            
            self.get_logger().info('')
            self.get_logger().info('╔═══════════════════════════════════════════════════════════╗')
            self.get_logger().info('║              ✅ WORKFLOW COMPLETED! ✅                    ║')
            self.get_logger().info('╚═══════════════════════════════════════════════════════════╝')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Workflow error: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    config_file = sys.argv[1] if len(sys.argv) > 1 else None
    
    node = ConfigurablePickPlace(config_file)
    
    print("\n⏸️  Press ENTER to start workflow...")
    input()
    
    try:
        node.execute_workflow()
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
