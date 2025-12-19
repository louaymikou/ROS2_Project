#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import NavigateToPose
from my_robot_controller.action import PickAndPlace
import time

class PickAndPlaceActionServer(Node):
    def __init__(self):
        super().__init__('pick_and_place_action_server')
        
        # Action server
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'pick_and_place',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Navigation action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Arm and gripper publishers
        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )
        
        self.get_logger().info('Pick and Place Action Server Started')
    
    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info('Received pick and place goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a cancellation request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute the pick and place mission."""
        self.get_logger().info('Executing pick and place mission...')
        
        feedback_msg = PickAndPlace.Feedback()
        result = PickAndPlace.Result()
        
        try:
            # Step 1: Navigate to pickup location
            feedback_msg.current_state = 'navigating_to_pickup'
            feedback_msg.progress = 0.1
            goal_handle.publish_feedback(feedback_msg)
            
            success = await self.navigate_to_point(
                goal_handle.request.pickup_x,
                goal_handle.request.pickup_y
            )
            
            if not success:
                result.success = False
                result.message = 'Failed to navigate to pickup location'
                return result
            
            # Step 2: Position arm and pick object
            feedback_msg.current_state = 'picking'
            feedback_msg.progress = 0.4
            goal_handle.publish_feedback(feedback_msg)
            
            self.pick_object()
            time.sleep(3)  # Wait for arm movement
            
            # Step 3: Navigate to dropoff location
            feedback_msg.current_state = 'navigating_to_dropoff'
            feedback_msg.progress = 0.6
            goal_handle.publish_feedback(feedback_msg)
            
            success = await self.navigate_to_point(
                goal_handle.request.dropoff_x,
                goal_handle.request.dropoff_y
            )
            
            if not success:
                result.success = False
                result.message = 'Failed to navigate to dropoff location'
                return result
            
            # Step 4: Drop object
            feedback_msg.current_state = 'dropping'
            feedback_msg.progress = 0.9
            goal_handle.publish_feedback(feedback_msg)
            
            self.drop_object()
            time.sleep(3)  # Wait for arm movement
            
            # Mission complete
            feedback_msg.current_state = 'completed'
            feedback_msg.progress = 1.0
            goal_handle.publish_feedback(feedback_msg)
            
            goal_handle.succeed()
            result.success = True
            result.message = 'Pick and place mission completed successfully'
            
        except Exception as e:
            self.get_logger().error(f'Error during pick and place: {str(e)}')
            result.success = False
            result.message = f'Mission failed: {str(e)}'
            goal_handle.abort()
        
        return result
    
    async def navigate_to_point(self, x, y):
        """Navigate to a specific point using Nav2."""
        self.get_logger().info(f'Navigating to ({x}, {y})')
        
        # Wait for navigation action server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = x
        nav_goal.pose.pose.position.y = y
        nav_goal.pose.pose.position.z = 0.0
        nav_goal.pose.pose.orientation.w = 1.0
        
        # Send goal and wait for result
        send_goal_future = self._nav_client.send_goal_async(nav_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False
        
        self.get_logger().info('Navigation goal accepted')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result:
            self.get_logger().info('Navigation successful')
            return True
        else:
            self.get_logger().error('Navigation failed')
            return False
    
    def pick_object(self):
        """Control arm and gripper to pick up object."""
        self.get_logger().info('Picking up object...')
        
        # Lower arm to picking position
        arm_cmd = Float64MultiArray()
        arm_cmd.data = [0.5, -1.0, 0.0]  # shoulder, elbow, gripper_rotate
        self.arm_pub.publish(arm_cmd)
        
        time.sleep(2)
        
        # Close gripper
        gripper_cmd = Float64MultiArray()
        gripper_cmd.data = [-0.3, -0.3]  # Close both fingers
        self.gripper_pub.publish(gripper_cmd)
        
        time.sleep(1)
        
        # Lift arm
        arm_cmd.data = [0.0, 0.0, 0.0]  # Return to neutral
        self.arm_pub.publish(arm_cmd)
    
    def drop_object(self):
        """Control arm and gripper to drop object."""
        self.get_logger().info('Dropping object...')
        
        # Lower arm
        arm_cmd = Float64MultiArray()
        arm_cmd.data = [0.5, -1.0, 0.0]
        self.arm_pub.publish(arm_cmd)
        
        time.sleep(2)
        
        # Open gripper
        gripper_cmd = Float64MultiArray()
        gripper_cmd.data = [0.0, 0.0]  # Open fingers
        self.gripper_pub.publish(gripper_cmd)
        
        time.sleep(1)
        
        # Raise arm
        arm_cmd.data = [0.0, 0.0, 0.0]
        self.arm_pub.publish(arm_cmd)

def main(args=None):
    rclpy.init(args=args)
    action_server = PickAndPlaceActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
