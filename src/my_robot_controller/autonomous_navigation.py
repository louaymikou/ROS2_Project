#!/usr/bin/env python3
"""
Autonomous Navigation Script for ROS2 Nav2
This script demonstrates how to control robot navigation programmatically
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math


def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    """
    Creates a PoseStamped message for navigation goals.
    
    Args:
        navigator: BasicNavigator instance
        position_x: X coordinate in meters (in map frame)
        position_y: Y coordinate in meters (in map frame)
        orientation_z: Rotation around Z-axis in radians (yaw angle)
                      0 rad = East, 1.57 rad = North, 3.14 rad = West, -1.57 rad = South
    
    Returns:
        PoseStamped message ready to send to Nav2
    """
    # Convert yaw angle to quaternion using manual calculation
    # For 2D navigation (rotation around Z-axis only)
    q_x = 0.0
    q_y = 0.0
    q_z = math.sin(orientation_z / 2.0)
    q_w = math.cos(orientation_z / 2.0)
    
    pose = PoseStamped()
    pose.header.frame_id = 'map'  # Reference frame for the coordinates
    pose.header.stamp = navigator.get_clock().now().to_msg()  # Current timestamp
    
    # Set position
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0  # Ground level for 2D navigation
    
    # Set orientation (rotation)
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    
    return pose


def main():
    """
    Main navigation function
    Demonstrates single goal navigation and waypoint following
    """
    # Initialize ROS2 Python client library
    rclpy.init()
    
    # Create navigator object to interface with Nav2
    nav = BasicNavigator()

    # --- Set Initial Pose ---
    # Tell Nav2 where the robot is starting (initializes AMCL localization)
    # IMPORTANT: Adjust these coordinates to match your robot's actual starting position!
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # Wait for Nav2 to fully activate (all nodes ready)
    print("Waiting for Nav2 to activate...")
    nav.waitUntilNav2Active()
    print("Nav2 is ready!")

    # --- Send Single Navigation Goal ---
    print("\n--- Going to single goal ---")
    # Navigate to position (3.5, 1.0) with 90° rotation (1.57 radians)
    # MODIFY THESE VALUES to match your desired goal location
    goal_pose = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    nav.goToPose(goal_pose)

    # Monitor progress until task completes
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
    
    # Print final result
    result = nav.getResult()
    if result == nav.TaskResult.SUCCEEDED:
        print("✓ Goal reached successfully!")
    elif result == nav.TaskResult.CANCELED:
        print("✗ Goal was canceled!")
    elif result == nav.TaskResult.FAILED:
        print("✗ Goal failed!")

    # --- Follow Multiple Waypoints ---
    print("\n--- Following waypoints ---")
    
    # Define a sequence of waypoints
    # MODIFY THESE VALUES to match your desired waypoint locations
    goal_pose1 = create_pose_stamped(nav, 3.5, 1.5, 1.57)   # Waypoint 1
    goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)   # Waypoint 2
    goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, 1.57)   # Waypoint 3

    waypoints = [goal_pose1, goal_pose2, goal_pose3]
    nav.followWaypoints(waypoints)

    # Monitor waypoint following progress
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"Current waypoint: {feedback.current_waypoint + 1}/{len(waypoints)}")
    
    result = nav.getResult()
    if result == nav.TaskResult.SUCCEEDED:
        print("✓ All waypoints completed!")
    else:
        print(f"✗ Waypoint navigation result: {result}")

    # Shutdown ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
