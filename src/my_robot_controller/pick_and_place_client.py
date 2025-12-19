#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from my_robot_controller.action import PickAndPlace

def main():
    rclpy.init()
    
    node = rclpy.create_node('pick_and_place_client')
    action_client = ActionClient(node, PickAndPlace, 'pick_and_place')
    
    node.get_logger().info('Waiting for action server...')
    action_client.wait_for_server()
    
    # Create goal
    goal_msg = PickAndPlace.Goal()
    goal_msg.pickup_x = 2.0
    goal_msg.pickup_y = 2.0
    goal_msg.dropoff_x = -2.0
    goal_msg.dropoff_y = -2.0
    
    node.get_logger().info('Sending pick and place goal...')
    send_goal_future = action_client.send_goal_async(
        goal_msg,
        feedback_callback=lambda feedback: node.get_logger().info(
            f'State: {feedback.feedback.current_state}, Progress: {feedback.feedback.progress:.2f}'
        )
    )
    
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    
    if not goal_handle.accepted:
        node.get_logger().error('Goal rejected')
        return
    
    node.get_logger().info('Goal accepted, waiting for result...')
    
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    
    result = result_future.result().result
    
    if result.success:
        node.get_logger().info(f'Success: {result.message}')
    else:
        node.get_logger().error(f'Failed: {result.message}')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
