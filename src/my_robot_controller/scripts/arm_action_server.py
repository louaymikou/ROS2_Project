#!/usr/bin/env python3
"""
Arm Action Server - Wraps ros2_control's FollowJointTrajectory action
Provides simplified interface for arm movements with feedback
"""
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from my_robot_controller.action import MoveArm
import time


class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')
        
        callback_group = ReentrantCallbackGroup()
        
        # Action server for simplified arm control
        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            self.execute_callback,
            callback_group=callback_group
        )
        
        # Action client to ros2_control's arm controller
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=callback_group
        )
        
        # Subscribe to joint states for feedback
        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=callback_group
        )
        
        self._current_joint_positions = {}
        self.get_logger().info('Arm Action Server started')
    
    def joint_state_callback(self, msg):
        """Store current joint positions"""
        for name, position in zip(msg.name, msg.position):
            self._current_joint_positions[name] = position
    
    async def execute_callback(self, goal_handle):
        """Execute arm movement goal"""
        self.get_logger().info('Executing arm movement...')
        
        feedback_msg = MoveArm.Feedback()
        result = MoveArm.Result()
        
        # Wait for arm controller to be ready
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm controller not available')
            result.success = False
            result.message = 'Arm controller not available'
            goal_handle.abort()
            return result
        
        # Create trajectory goal
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = [
            'shoulder_joint',
            'elbow_joint',
            'gripper_rotate_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [
            float(goal_handle.request.shoulder_target),
            float(goal_handle.request.elbow_target),
            float(goal_handle.request.gripper_rotation_target)
        ]
        point.time_from_start.sec = int(goal_handle.request.duration)
        point.time_from_start.nanosec = int((goal_handle.request.duration % 1) * 1e9)
        
        trajectory_goal.trajectory.points = [point]
        
        # Send goal to arm controller
        self.get_logger().info(f'Sending arm goal: {point.positions}')
        send_goal_future = self._arm_client.send_goal_async(trajectory_goal)
        
        try:
            goal_response = await send_goal_future
            if not goal_response.accepted:
                self.get_logger().error('Arm goal rejected')
                result.success = False
                result.message = 'Goal rejected by arm controller'
                goal_handle.abort()
                return result
            
            self.get_logger().info('Arm goal accepted, executing...')
            
            # Get result future
            get_result_future = goal_response.get_result_async()
            
            # Publish feedback while executing
            start_time = time.time()
            while not get_result_future.done():
                await rclpy.task.sleep(0.1)
                
                # Calculate progress
                elapsed = time.time() - start_time
                progress = min(100.0, (elapsed / goal_handle.request.duration) * 100.0)
                
                feedback_msg.progress_percent = progress
                feedback_msg.current_positions = [
                    self._current_joint_positions.get('shoulder_joint', 0.0),
                    self._current_joint_positions.get('elbow_joint', 0.0),
                    self._current_joint_positions.get('gripper_rotate_joint', 0.0)
                ]
                feedback_msg.status = f'Moving arm... {progress:.1f}%'
                
                goal_handle.publish_feedback(feedback_msg)
                
                # Check if goal was cancelled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Arm movement cancelled')
                    result.success = False
                    result.message = 'Cancelled by user'
                    return result
            
            # Get final result
            trajectory_result = await get_result_future
            
            if trajectory_result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().info('Arm movement completed successfully')
                result.success = True
                result.final_positions = feedback_msg.current_positions
                result.message = 'Arm reached target position'
                goal_handle.succeed()
            else:
                self.get_logger().warn(f'Arm movement failed: {trajectory_result.result.error_string}')
                result.success = False
                result.message = f'Failed: {trajectory_result.result.error_string}'
                goal_handle.abort()
            
        except Exception as e:
            self.get_logger().error(f'Arm movement exception: {str(e)}')
            result.success = False
            result.message = f'Exception: {str(e)}'
            goal_handle.abort()
        
        return result


def main(args=None):
    rclpy.init(args=args)
    arm_action_server = ArmActionServer()
    
    try:
        rclpy.spin(arm_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        arm_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
