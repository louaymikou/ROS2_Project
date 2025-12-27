#!/usr/bin/env python3
"""
Ultrasonic Aggregator Node
==========================
Combine 8 ultrasonic Range sensors into a single LaserScan message
for Nav2 obstacle avoidance compatibility.

This node subscribes to 8 individual Range topics from HC-SR04 sensors
and publishes a synthetic LaserScan that Nav2 can use for local costmap.

Sensors arrangement (45° apart):
  - ultrasonic_front       (0°)
  - ultrasonic_front_left  (45°)
  - ultrasonic_left        (90°)
  - ultrasonic_rear_left   (135°)
  - ultrasonic_rear        (180°)
  - ultrasonic_rear_right  (225° / -135°)
  - ultrasonic_right       (270° / -90°)
  - ultrasonic_front_right (315° / -45°)

Output: /ultrasonic_scan (sensor_msgs/LaserScan)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range, LaserScan
import math
import time


class UltrasonicAggregator(Node):
    
    def __init__(self):
        super().__init__('ultrasonic_aggregator')
        
        # Parameters
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 4.0)
        self.declare_parameter('interpolation_samples', 5)
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.interp_samples = self.get_parameter('interpolation_samples').value
        
        # Sensor configuration: name -> angle (radians)
        self.sensors = {
            'ultrasonic_front': 0.0,
            'ultrasonic_front_left': math.pi / 4,        # 45°
            'ultrasonic_left': math.pi / 2,              # 90°
            'ultrasonic_rear_left': 3 * math.pi / 4,     # 135°
            'ultrasonic_rear': math.pi,                  # 180°
            'ultrasonic_rear_right': -3 * math.pi / 4,   # -135° (225°)
            'ultrasonic_right': -math.pi / 2,            # -90° (270°)
            'ultrasonic_front_right': -math.pi / 4,      # -45° (315°)
        }
        
        # Storage for latest readings
        self.readings = {name: float('inf') for name in self.sensors}
        self.timestamps = {name: 0.0 for name in self.sensors}
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to each ultrasonic sensor
        self.subscriptions_list = []
        for sensor_name in self.sensors:
            topic = f'/ultrasonic/{sensor_name}'
            sub = self.create_subscription(
                Range,
                topic,
                lambda msg, name=sensor_name: self.range_callback(msg, name),
                sensor_qos
            )
            self.subscriptions_list.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')
        
        # Publisher for aggregated LaserScan
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/ultrasonic_scan',
            10
        )
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_scan)
        
        self.get_logger().info('Ultrasonic Aggregator initialized')
        self.get_logger().info(f'Publishing LaserScan at {self.publish_rate} Hz')
    
    def range_callback(self, msg: Range, sensor_name: str):
        """Store the latest range reading from a sensor."""
        # Validate range
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.readings[sensor_name] = msg.range
        else:
            self.readings[sensor_name] = float('inf')
        
        self.timestamps[sensor_name] = time.time()
    
    def publish_scan(self):
        """Publish aggregated LaserScan message."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        
        # Full 360° scan with resolution matching sensor spacing
        # 8 sensors at 45° intervals = we create a scan with interpolation
        num_readings = 72  # 5° resolution for smooth costmap
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / num_readings
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.publish_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Initialize ranges with max value
        scan.ranges = [self.range_max] * num_readings
        scan.intensities = [0.0] * num_readings
        
        # Map sensor readings to scan indices with interpolation
        for sensor_name, angle in self.sensors.items():
            reading = self.readings[sensor_name]
            
            # Check if reading is recent (within 0.5 seconds)
            if time.time() - self.timestamps[sensor_name] > 0.5:
                reading = self.range_max
            
            # Calculate scan index for this sensor angle
            # Convert angle to scan index
            normalized_angle = angle
            if normalized_angle < scan.angle_min:
                normalized_angle += 2 * math.pi
            
            index = int((normalized_angle - scan.angle_min) / scan.angle_increment) % num_readings
            
            # Apply reading with interpolation to neighboring indices
            half_spread = self.interp_samples // 2
            for offset in range(-half_spread, half_spread + 1):
                idx = (index + offset) % num_readings
                # Weight decreases with distance from center
                weight = 1.0 - abs(offset) / (half_spread + 1)
                
                # Take minimum of existing and weighted new reading
                weighted_reading = reading + (1 - weight) * (self.range_max - reading)
                scan.ranges[idx] = min(scan.ranges[idx], weighted_reading)
                
                # Set intensity based on confidence
                if reading < self.range_max:
                    scan.intensities[idx] = max(scan.intensities[idx], weight * 100)
        
        self.scan_pub.publish(scan)
    
    def get_diagnostics(self):
        """Return diagnostic information about sensor status."""
        current_time = time.time()
        status = {}
        for sensor_name in self.sensors:
            age = current_time - self.timestamps[sensor_name]
            status[sensor_name] = {
                'reading': self.readings[sensor_name],
                'age': age,
                'status': 'OK' if age < 0.5 else 'STALE'
            }
        return status


def main(args=None):
    rclpy.init(args=args)
    
    node = UltrasonicAggregator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
