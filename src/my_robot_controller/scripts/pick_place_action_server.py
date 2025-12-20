#!/usr/bin/env python3
"""
Pick and Place Action Server - Coordinates navigation and arm manipulation
Executes complete pick or place sequences
"""
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from my_robot_controller.action import PickPlace
from sensor_msgs.msg import JointState
import time
import math


class PickPlaceActionServer(Node):
    def __init__(self):
        super().__init__('pick_place_action_server')
        
        callback_group = ReentrantCallbackGroup()
        
        # Action server for pick/place operations
        self._action_server = ActionServer(
            self,
            PickPlace,
            'pick_place',
            self.execute_callback,
            callback_group=callback_group
        )
        
        # Action clients
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=callback_group
        )
        
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=callback_group
        )
        
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=callback_group
        )
        
        # Subscribe to joint states
        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=callback_group
        )
        
        self._current_joints = {}
        
        # Arm configurations
        self.ARM_HOME = [0.0, 0.0, 0.0]  # Retracted, safe for navigation
        self.ARM_PICK_APPROACH = [0.5, 0.5, 0.0]  # Approaching object
        self.ARM_PICK_GRASP = [0.75, 0.75, 0.0]  # At grasping height
        self.ARM_PICK_LIFT = [0.0, 0.0, 0.0]  # Lifted with object
        
        self.GRIPPER_OPEN = -0.12
        self.GRIPPER_CLOSED = 0.0
        
        self.get_logger().info('Pick and Place Action Server started')
    
    def joint_state_callback(self, msg):
        """Store current joint states"""
        for name, position in zip(msg.name, msg.position):
            self._current_joints[name] = position
    
    async def execute_callback(self, goal_handle):
        """Execute pick or place operation"""
        self.get_logger().info(f'Executing {goal_handle.request.operation} operation at {goal_handle.request.target_position}')
        
        feedback_msg = PickPlace.Feedback()
        result = PickPlace.Result()
        start_time = time.time()
        
        try:
            if goal_handle.request.operation == "pick":
                success = await self.execute_pick_sequence(goal_handle, feedback_msg)
            elif goal_handle.request.operation == "place":
                success = await self.execute_place_sequence(goal_handle, feedback_msg)
            else:
                self.get_logger().error(f'Unknown operation: {goal_handle.request.operation}')
                result.success = False
                result.message = f'Unknown operation: {goal_handle.request.operation}'
                goal_handle.abort()
                return result
            
            result.execution_time = time.time() - start_time
            
            if success:
                result.success = True
                result.message = f'{goal_handle.request.operation} operation completed successfully'
                self.get_logger().info(result.message)
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f'{goal_handle.request.operation} operation failed'
                self.get_logger().warn(result.message)
                goal_handle.abort()
                
        except Exception as e:
            self.get_logger().error(f'Pick/Place exception: {str(e)}')
            result.success = False
            result.message = f'Exception: {str(e)}'
            result.execution_time = time.time() - start_time
            goal_handle.abort()
        
        return result
    
    async def execute_pick_sequence(self, goal_handle, feedback_msg):
        """Execute complete pick sequence"""
        
        # Phase 1: Navigate to approach position (0.5m before target)
        feedback_msg.current_phase = "navigating"
        feedback_msg.progress_percent = 10.0
        goal_handle.publish_feedback(feedback_msg)
        
        approach_pose = self.create_approach_pose(goal_handle.request.target_position, offset=-0.5)
        if not await self.navigate_to_pose(approach_pose):
            return False
        
        # Phase 2: Open gripper
        feedback_msg.current_phase = "opening_gripper"
        feedback_msg.progress_percent = 30.0
        goal_handle.publish_feedback(feedback_msg)
        
        if not await self.control_gripper(self.GRIPPER_OPEN):
            return False
        
        # Phase 3: Extend arm to pre-grasp position
        feedback_msg.current_phase = "approaching"
        feedback_msg.progress_percent = 40.0
        goal_handle.publish_feedback(feedback_msg)
        
        if not await self.move_arm(self.ARM_PICK_APPROACH, 2.0):
            return False
        
        # Phase 4: Lower arm to grasping position
        feedback_msg.current_phase = "lowering_arm"
        feedback_msg.progress_percent = 60.0
        goal_handle.publish_feedback(feedback_msg)
        
        if not await self.move_arm(self.ARM_PICK_GRASP, 2.0):
            return False
        
        # Phase 5: Close gripper (grasp object)
        feedback_msg.current_phase = "grasping"
        feedback_msg.progress_percent = 75.0
        goal_handle.publish_feedback(feedback_msg)
        
        await rclpy.task.sleep(0.5)  # Wait for arm to stabilize
        
        if not await self.control_gripper(self.GRIPPER_CLOSED):
            return False
        
        await rclpy.task.sleep(1.0)  # Ensure firm grasp
        
        # Phase 6: Lift arm (retract to home)
        feedback_msg.current_phase = "lifting"
        feedback_msg.progress_percent = 90.0
        goal_handle.publish_feedback(feedback_msg)
        
        if not await self.move_arm(self.ARM_HOME, 2.0):
            return False
        
        feedback_msg.progress_percent = 100.0
        goal_handle.publish_feedback(feedback_msg)
        
        return True
    
    async def execute_place_sequence(self, goal_handle, feedback_msg):
        """Execute complete place sequence"""
        
        # Phase 1: Navigate to drop-off position
        feedback_msg.current_phase = "navigating"
        feedback_msg.progress_percent = 10.0
        goal_handle.publish_feedback(feedback_msg)
        
        place_pose = self.create_approach_pose(goal_handle.request.target_position, offset=-0.5)
        if not await self.navigate_to_pose(place_pose):
            return False
        
        # Phase 2: Extend arm to placing height
        feedback_msg.current_phase = "lowering_arm"
        feedback_msg.progress_percent = 40.0
        goal_handle.publish_feedback(feedback_msg)
        
        if not await self.move_arm(self.ARM_PICK_GRASP, 2.0):
            return False
        
        await rclpy.task.sleep(0.5)
        
        # Phase 3: Open gripper (release object)
        feedback_msg.current_phase = "placing"
        feedback_msg.progress_percent = 70.0
        goal_handle.publish_feedback(feedback_msg)
        
        if not await self.control_gripper(self.GRIPPER_OPEN):
            return False
        
        await rclpy.task.sleep(1.0)  # Let object settle
        
        # Phase 4: Retract arm
        feedback_msg.current_phase = "lifting"
        feedback_msg.progress_percent = 90.0
        goal_handle.publish_feedback(feedback_msg)
        
        if not await self.move_arm(self.ARM_HOME, 2.0):
            return False
        
        feedback_msg.progress_percent = 100.0
        goal_handle.publish_feedback(feedback_msg)
        
        return True
    
    def create_approach_pose(self, target: Point, offset: float = 0.0):
        """Create a PoseStamped for navigation with optional offset"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target.x + offset
        pose.pose.position.y = target.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # Facing forward
        return pose
    
    async def navigate_to_pose(self, goal_pose: PoseStamped):
        """Navigate to target pose using Nav2"""
        self.get_logger().info(f'Navigating to ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available')
            return False
        
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        send_goal_future = self._nav_client.send_goal_async(nav_goal)
        goal_response = await send_goal_future
        
        if not goal_response.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False
        
        self.get_logger().info('Navigation goal accepted, waiting for completion...')
        result_future = goal_response.get_result_async()
        result = await result_future
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation completed successfully')
            return True
        else:
            self.get_logger().warn(f'Navigation failed with status: {result.status}')
            return False
    
    async def move_arm(self, joint_positions, duration=2.0):
        """Move arm to specified joint positions"""
        self.get_logger().info(f'Moving arm to {joint_positions}')
        
        if not self._arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Arm controller not available')
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_rotate_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        goal.trajectory.points = [point]
        
        send_goal_future = self._arm_client.send_goal_async(goal)
        goal_response = await send_goal_future
        
        if not goal_response.accepted:
            self.get_logger().error('Arm goal rejected')
            return False
        
        result_future = goal_response.get_result_async()
        result = await result_future
        
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    
    async def control_gripper(self, position):
        """Open or close gripper"""
        self.get_logger().info(f'Setting gripper to {position}')
        
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper controller not available')
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = 1
        
        goal.trajectory.points = [point]
        
        send_goal_future = self._gripper_client.send_goal_async(goal)
        goal_response = await send_goal_future
        
        if not goal_response.accepted:
            return False
        
        result_future = goal_response.get_result_async()
        result = await result_future
        
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def main(args=None):
    rclpy.init(args=args)
    pick_place_server = PickPlaceActionServer()
    
    try:
        rclpy.spin(pick_place_server)
    except KeyboardInterrupt:
        pass
    finally:
        pick_place_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
