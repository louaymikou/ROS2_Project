#!/usr/bin/env python3
"""
Complete Pick and Place Workflow
Combines Nav2 navigation with MoveIt arm control for autonomous box handling
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from action_msgs.msg import GoalStatus
import time
import math


class PickAndPlaceWorkflow(Node):
    def __init__(self):
        super().__init__('pick_and_place_workflow')
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║       PICK AND PLACE AUTONOMOUS WORKFLOW                ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
        # Navigation action client
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Initialize MoveIt
        self.get_logger().info('🤖 Initializing MoveIt...')
        try:
            self.moveit = MoveItPy(node=self)
            self.arm = self.moveit.get_planning_component("arm")
            self.gripper = self.moveit.get_planning_component("gripper")
            self.get_logger().info('✅ MoveIt initialized successfully!')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize MoveIt: {e}')
            raise
        
        # Wait for Nav2 action server
        self.get_logger().info('⏳ Waiting for Nav2 action server...')
        self._nav_action_client.wait_for_server()
        self.get_logger().info('✅ Nav2 ready!')
        
        # Current goal handle
        self._goal_handle = None
        self._goal_result = None
        
        # Define waypoints (you can customize these)
        self.home_position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.point_a = {'x': 2.0, 'y': 1.0, 'yaw': 0.0}  # Pick location
        self.point_b = {'x': -2.0, 'y': -1.0, 'yaw': 1.57}  # Place location
        
    def set_initial_pose(self, x, y, yaw):
        """Set initial pose for AMCL"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Set covariance
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'📍 Initial pose set: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')
    
    def navigate_to(self, x, y, yaw, description="target"):
        """Navigate to a specific pose"""
        self.get_logger().info(f'🧭 Navigating to {description}: ({x:.2f}, {y:.2f})')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Send goal
        send_goal_future = self._nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Navigation goal rejected!')
            return False
        
        self.get_logger().info('✅ Navigation goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ Arrived at {description}!')
            return True
        else:
            self.get_logger().error(f'❌ Navigation to {description} failed!')
            return False
    
    def nav_feedback_callback(self, feedback_msg):
        """Navigation feedback callback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        if int(distance * 10) % 10 == 0:  # Log every meter
            self.get_logger().info(f'   Distance remaining: {distance:.2f}m')
    
    def move_arm_to_pose(self, pose_name):
        """Move arm to predefined pose"""
        self.get_logger().info(f'🦾 Moving arm to: {pose_name}')
        
        try:
            self.arm.set_goal_state(configuration_name=pose_name)
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info('   Planning successful, executing...')
                robot_trajectory = plan_result.trajectory
                self.moveit.execute(robot_trajectory, controllers=[])
                self.get_logger().info(f'✅ Arm moved to {pose_name}')
                return True
            else:
                self.get_logger().error(f'❌ Failed to plan arm movement to {pose_name}')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ Arm movement error: {e}')
            return False
    
    def move_gripper(self, state):
        """Control gripper: 'open' or 'closed'"""
        self.get_logger().info(f'🤏 Gripper: {state}')
        
        try:
            self.gripper.set_goal_state(configuration_name=state)
            plan_result = self.gripper.plan()
            
            if plan_result:
                robot_trajectory = plan_result.trajectory
                self.moveit.execute(robot_trajectory, controllers=[])
                self.get_logger().info(f'✅ Gripper {state}')
                return True
            else:
                self.get_logger().error(f'❌ Failed to {state} gripper')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ Gripper error: {e}')
            return False
    
    def pick_sequence(self):
        """Execute pick sequence"""
        self.get_logger().info('')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('        PICKING SEQUENCE START')
        self.get_logger().info('═══════════════════════════════════════')
        
        # 1. Open gripper
        if not self.move_gripper("open"):
            return False
        time.sleep(1)
        
        # 2. Move to ready position
        if not self.move_arm_to_pose("ready"):
            return False
        time.sleep(1)
        
        # 3. Move to extended position (reach object)
        if not self.move_arm_to_pose("extended"):
            return False
        time.sleep(1)
        
        # 4. Close gripper (grasp)
        self.get_logger().info('🎯 Grasping object...')
        if not self.move_gripper("closed"):
            return False
        time.sleep(1)
        
        # 5. Lift object
        if not self.move_arm_to_pose("ready"):
            return False
        time.sleep(1)
        
        # 6. Tuck arm for safe transport
        if not self.move_arm_to_pose("tucked"):
            return False
        
        self.get_logger().info('✅ Pick sequence completed!')
        return True
    
    def place_sequence(self):
        """Execute place sequence"""
        self.get_logger().info('')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('        PLACING SEQUENCE START')
        self.get_logger().info('═══════════════════════════════════════')
        
        # 1. Move to ready position
        if not self.move_arm_to_pose("ready"):
            return False
        time.sleep(1)
        
        # 2. Extend to place location
        if not self.move_arm_to_pose("extended"):
            return False
        time.sleep(1)
        
        # 3. Release object
        self.get_logger().info('📦 Releasing object...')
        if not self.move_gripper("open"):
            return False
        time.sleep(1)
        
        # 4. Retract arm
        if not self.move_arm_to_pose("ready"):
            return False
        time.sleep(1)
        
        # 5. Return to home position
        if not self.move_arm_to_pose("home"):
            return False
        
        self.get_logger().info('✅ Place sequence completed!')
        return True
    
    def execute_full_workflow(self):
        """Execute complete pick and place workflow"""
        self.get_logger().info('')
        self.get_logger().info('╔═══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║     STARTING COMPLETE PICK AND PLACE WORKFLOW            ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        
        # Step 0: Initialize arm to home position
        self.get_logger().info('📍 STEP 0: Initialize - Moving arm to home position')
        if not self.move_arm_to_pose("home"):
            self.get_logger().error('❌ Failed to initialize arm')
            return False
        time.sleep(2)
        
        # Step 1: Navigate to Point A (Pick location)
        self.get_logger().info('')
        self.get_logger().info('📍 STEP 1: Navigate to Point A (Pick Location)')
        if not self.navigate_to(
            self.point_a['x'], 
            self.point_a['y'], 
            self.point_a['yaw'],
            "Point A (Pick)"
        ):
            self.get_logger().error('❌ Failed to reach Point A')
            return False
        time.sleep(2)
        
        # Step 2: Execute pick sequence
        self.get_logger().info('')
        self.get_logger().info('📍 STEP 2: Pick Object')
        if not self.pick_sequence():
            self.get_logger().error('❌ Failed to pick object')
            return False
        time.sleep(2)
        
        # Step 3: Navigate to Point B (Place location)
        self.get_logger().info('')
        self.get_logger().info('📍 STEP 3: Navigate to Point B (Place Location)')
        if not self.navigate_to(
            self.point_b['x'], 
            self.point_b['y'], 
            self.point_b['yaw'],
            "Point B (Place)"
        ):
            self.get_logger().error('❌ Failed to reach Point B')
            return False
        time.sleep(2)
        
        # Step 4: Execute place sequence
        self.get_logger().info('')
        self.get_logger().info('📍 STEP 4: Place Object')
        if not self.place_sequence():
            self.get_logger().error('❌ Failed to place object')
            return False
        time.sleep(2)
        
        # Step 5: Return to home position
        self.get_logger().info('')
        self.get_logger().info('📍 STEP 5: Return to Home Position')
        if not self.navigate_to(
            self.home_position['x'], 
            self.home_position['y'], 
            self.home_position['yaw'],
            "Home"
        ):
            self.get_logger().error('❌ Failed to return home')
            return False
        
        # Final: Ensure arm is at home
        self.get_logger().info('')
        self.get_logger().info('📍 FINAL: Ensure arm at home position')
        self.move_arm_to_pose("home")
        
        self.get_logger().info('')
        self.get_logger().info('╔═══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║         ✅ WORKFLOW COMPLETED SUCCESSFULLY! ✅            ║')
        self.get_logger().info('╚═══════════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    workflow = PickAndPlaceWorkflow()
    
    print("\n" + "="*60)
    print("  PICK AND PLACE AUTONOMOUS WORKFLOW")
    print("="*60)
    print("\nThis workflow will:")
    print("  1. Start from home position")
    print("  2. Navigate to Point A and pick object")
    print("  3. Navigate to Point B and place object")
    print("  4. Return to home position")
    print("\n" + "="*60)
    
    input("\n⏸️  Press ENTER to start the workflow...")
    
    try:
        success = workflow.execute_full_workflow()
        
        if success:
            workflow.get_logger().info('🎉 Mission accomplished!')
        else:
            workflow.get_logger().error('❌ Workflow failed!')
        
    except KeyboardInterrupt:
        workflow.get_logger().info('\n⚠️  Workflow interrupted by user')
    except Exception as e:
        workflow.get_logger().error(f'❌ Workflow error: {e}')
    finally:
        workflow.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
