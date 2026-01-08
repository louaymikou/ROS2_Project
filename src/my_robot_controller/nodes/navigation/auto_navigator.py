#!/usr/bin/env python3
"""
Auto Navigator - Complete autonomous navigation workflow
Launches Gazebo + Nav2 and provides navigation capabilities
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import time
import math

class AutoNavigator(Node):
    def __init__(self):
        super().__init__('auto_navigator')
        
        # Action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for initial pose (for AMCL)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║           AUTONOMOUS NAVIGATOR - NAV2 READY             ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
        # Wait for Nav2 action server
        self.get_logger().info('⏳ Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('✅ Nav2 action server ready!')
        
        # Current goal handle
        self._goal_handle = None
        self._goal_result = None
        
    def set_initial_pose(self, x=0.0, y=0.0, yaw=0.0):
        """Set initial pose for AMCL localization"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance (uncertainty)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25  # x variance
        msg.pose.covariance[7] = 0.25  # y variance
        msg.pose.covariance[35] = 0.06853  # yaw variance
        
        self.get_logger().info('═' * 60)
        self.get_logger().info('📍 Setting initial pose')
        self.get_logger().info(f'   Position: x={x:.2f}m, y={y:.2f}m')
        self.get_logger().info(f'   Orientation: yaw={yaw:.2f}rad ({math.degrees(yaw):.1f}°)')
        self.get_logger().info('═' * 60)
        
        # Publish multiple times to ensure AMCL receives it
        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.1)
            
        time.sleep(2)  # Wait for AMCL to process
        
    def create_goal(self, x, y, yaw=0.0):
        """Create a navigation goal"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal_msg
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by Nav2!')
            return
        
        self.get_logger().info('✅ Goal accepted by Nav2')
        self.get_logger().info('🤖 Robot is navigating...')
        
        # Wait for result
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        self._goal_result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Goal reached successfully!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('❌ Goal aborted!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('⚠️  Goal canceled!')
        else:
            self.get_logger().error(f'❌ Goal failed with status: {status}')
    
    def navigate_to(self, x, y, yaw=0.0, description=""):
        """Navigate to a specific pose"""
        goal_msg = self.create_goal(x, y, yaw)
        
        self.get_logger().info('')
        self.get_logger().info('═' * 60)
        if description:
            self.get_logger().info(f'🎯 Navigation Goal: {description}')
        else:
            self.get_logger().info('🎯 New Navigation Goal')
        self.get_logger().info(f'   Target: x={x:.2f}m, y={y:.2f}m, yaw={math.degrees(yaw):.1f}°')
        self.get_logger().info('═' * 60)
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Wait for completion
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        
        if self._goal_handle:
            rclpy.spin_until_future_complete(self, self._result_future)
            return True
        return False
    
    def run_demo_navigation(self):
        """Run a demo navigation sequence through multiple waypoints"""
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║        STARTING DEMO NAVIGATION SEQUENCE                ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
        # Set initial pose (assuming robot starts at origin)
        self.set_initial_pose(0.0, 0.0, 0.0)
        
        # Define waypoints (x, y, yaw, description)
        waypoints = [
            (1.0, 0.0, 0.0, "Move forward 1m"),
            (1.0, 1.0, 1.57, "Move to (1, 1), face North"),
            (0.0, 1.0, 3.14, "Move to (0, 1), face West"),
            (0.0, 0.0, 0.0, "Return to origin, face East"),
        ]
        
        self.get_logger().info(f'📋 Navigation plan: {len(waypoints)} waypoints')
        time.sleep(3)
        
        # Navigate through waypoints
        for i, (x, y, yaw, desc) in enumerate(waypoints, 1):
            self.get_logger().info(f'\n🔹 Waypoint {i}/{len(waypoints)}')
            success = self.navigate_to(x, y, yaw, desc)
            
            if not success:
                self.get_logger().error(f'❌ Failed to reach waypoint {i}')
                break
            
            time.sleep(2)  # Pause between waypoints
        
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║          DEMO NAVIGATION SEQUENCE COMPLETED             ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
    
    def interactive_navigation(self):
        """Interactive mode - user enters coordinates"""
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║          INTERACTIVE NAVIGATION MODE                    ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        
        # Set initial pose
        self.get_logger().info('\n📍 Set initial pose (press Enter for default 0,0,0)')
        try:
            x_input = input('Initial X [0.0]: ')
            x = float(x_input) if x_input else 0.0
            y_input = input('Initial Y [0.0]: ')
            y = float(y_input) if y_input else 0.0
            yaw_input = input('Initial Yaw in degrees [0.0]: ')
            yaw = math.radians(float(yaw_input)) if yaw_input else 0.0
            
            self.set_initial_pose(x, y, yaw)
        except ValueError:
            self.get_logger().error('Invalid input, using default (0, 0, 0)')
            self.set_initial_pose(0.0, 0.0, 0.0)
        
        # Navigate to goals
        while True:
            try:
                self.get_logger().info('\n🎯 Enter navigation goal (or "quit" to exit)')
                goal_input = input('Goal X (or quit): ')
                
                if goal_input.lower() in ['quit', 'q', 'exit']:
                    break
                
                x = float(goal_input)
                y = float(input('Goal Y: '))
                yaw_input = input('Goal Yaw in degrees [0.0]: ')
                yaw = math.radians(float(yaw_input)) if yaw_input else 0.0
                
                self.navigate_to(x, y, yaw)
                
            except (ValueError, EOFError, KeyboardInterrupt):
                break
        
        self.get_logger().info('👋 Exiting interactive navigation')


def main(args=None):
    rclpy.init(args=args)
    navigator = AutoNavigator()
    
    try:
        # Check command line arguments for mode
        import sys
        if len(sys.argv) > 1:
            mode = sys.argv[1]
            if mode == 'demo':
                navigator.run_demo_navigation()
            elif mode == 'interactive':
                navigator.interactive_navigation()
            else:
                navigator.get_logger().error(f'Unknown mode: {mode}')
                navigator.get_logger().info('Usage: auto_navigator.py [demo|interactive]')
        else:
            # Default: run demo
            navigator.run_demo_navigation()
            
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation interrupted by user')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
