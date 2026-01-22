#!/usr/bin/env python3
"""
Laser scan normalizer to fix variable reading counts for slam_toolbox.
This node normalizes all scans to have a consistent number of readings (226)
by interpolating or padding/truncating as needed.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanNormalizer(Node):
    def __init__(self):
        super().__init__('laser_scan_normalizer')
        
        # Declare parameter for target number of readings
        self.declare_parameter('target_readings', 226)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_normalized')
        
        target_readings = self.get_parameter('target_readings').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Subscribe to the original scan
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )
        
        # Publish the normalized scan
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            10
        )
        
        self.target_readings = target_readings
        self.get_logger().info(
            f'Laser scan normalizer started: {input_topic} -> {output_topic} '
            f'(normalizing to {target_readings} readings)'
        )
    
    def scan_callback(self, msg):
        actual_readings = len(msg.ranges)
        
        # Create a copy of the message
        normalized_msg = LaserScan()
        normalized_msg.header = msg.header
        normalized_msg.angle_min = msg.angle_min
        normalized_msg.angle_max = msg.angle_max
        normalized_msg.angle_increment = msg.angle_increment
        normalized_msg.time_increment = msg.time_increment
        normalized_msg.scan_time = msg.scan_time
        normalized_msg.range_min = msg.range_min
        normalized_msg.range_max = msg.range_max
        
        if actual_readings == self.target_readings:
            # Already correct size, just copy
            normalized_msg.ranges = list(msg.ranges)
            normalized_msg.intensities = list(msg.intensities) if msg.intensities else []
        elif actual_readings > self.target_readings:
            # Too many readings - downsample by selecting evenly spaced indices
            step = (actual_readings - 1) / (self.target_readings - 1) if self.target_readings > 1 else 0
            normalized_msg.ranges = []
            normalized_msg.intensities = []
            
            for i in range(self.target_readings):
                idx = int(round(i * step))
                if idx >= actual_readings:
                    idx = actual_readings - 1
                normalized_msg.ranges.append(msg.ranges[idx])
                if msg.intensities and len(msg.intensities) > idx:
                    normalized_msg.intensities.append(msg.intensities[idx])
        else:
            # Too few readings - upsample by linear interpolation
            normalized_msg.ranges = []
            normalized_msg.intensities = []
            
            if actual_readings == 0:
                # No readings - fill with max range
                normalized_msg.ranges = [msg.range_max] * self.target_readings
                normalized_msg.intensities = [0.0] * self.target_readings if msg.intensities else []
            elif actual_readings == 1:
                # Single reading - replicate it
                normalized_msg.ranges = [msg.ranges[0]] * self.target_readings
                normalized_msg.intensities = [msg.intensities[0] if msg.intensities else 0.0] * self.target_readings
            else:
                # Linear interpolation
                step = (actual_readings - 1) / (self.target_readings - 1) if self.target_readings > 1 else 0
                for i in range(self.target_readings):
                    pos = i * step
                    idx_low = int(pos)
                    idx_high = min(idx_low + 1, actual_readings - 1)
                    alpha = pos - idx_low
                    
                    # Interpolate range
                    range_val = msg.ranges[idx_low] * (1 - alpha) + msg.ranges[idx_high] * alpha
                    normalized_msg.ranges.append(range_val)
                    
                    # Interpolate intensity if available
                    if msg.intensities and len(msg.intensities) > idx_high:
                        intensity_val = msg.intensities[idx_low] * (1 - alpha) + msg.intensities[idx_high] * alpha
                        normalized_msg.intensities.append(intensity_val)
                    elif msg.intensities:
                        normalized_msg.intensities.append(msg.intensities[0] if len(msg.intensities) > 0 else 0.0)
        
        # Adjust angle_increment to match the new number of readings
        # This ensures the scan geometry is correct
        angle_span = msg.angle_max - msg.angle_min
        normalized_msg.angle_increment = angle_span / (self.target_readings - 1) if self.target_readings > 1 else msg.angle_increment
        
        # Publish the normalized scan
        self.publisher.publish(normalized_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanNormalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
