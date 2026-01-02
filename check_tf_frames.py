#!/usr/bin/env python3
"""
Script to verify TF frames are correctly set up for laser scanning
Run this after launching your robot to check if the laser frame is properly connected
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

class TFChecker(Node):
    def __init__(self):
        super().__init__('tf_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for TF to populate
        time.sleep(2.0)
        
        # Check critical transforms
        self.check_transform('base_link', 'laser_frame')
        self.check_transform('odom', 'base_link')
        self.check_transform('odom', 'laser_frame')
        
        # List all available frames
        self.get_logger().info('\n=== Available Frames ===')
        frames = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(f'\n{frames}')
        
    def check_transform(self, parent_frame, child_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            self.get_logger().info(
                f'\n✓ Transform {parent_frame} -> {child_frame}:\n'
                f'  Translation: x={trans.x:.3f}, y={trans.y:.3f}, z={trans.z:.3f}\n'
                f'  Rotation: x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}'
            )
            return True
            
        except TransformException as ex:
            self.get_logger().error(
                f'\n✗ Failed to get transform {parent_frame} -> {child_frame}: {ex}'
            )
            return False

def main(args=None):
    rclpy.init(args=args)
    
    print('\n' + '='*60)
    print('  TF Frame Checker - Laser Scan Verification')
    print('='*60)
    
    checker = TFChecker()
    
    print('\n' + '='*60)
    print('  Check Complete!')
    print('='*60 + '\n')
    
    checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
