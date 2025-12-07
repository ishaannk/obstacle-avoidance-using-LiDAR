#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
from collections import deque

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Declare parameters with defaults
        self.declare_parameter('min_distance_threshold', 1.0)  # meters
        self.declare_parameter('fov_start_angle', 0.0)  # degrees
        self.declare_parameter('fov_end_angle', 120.0)  # degrees
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('min_obstacle_points', 5)  # minimum points to confirm obstacle
        self.declare_parameter('consecutive_detections', 3)  # require N consecutive scans
        self.declare_parameter('account_rplidar_transform', True)  # account for RPLIDAR angle transform
        self.declare_parameter('exclude_angle_ranges', [])  # List of [start, end] angle ranges to exclude (degrees)
        
        # Get parameters
        self.min_distance = self.get_parameter('min_distance_threshold').value
        self.fov_start = math.radians(self.get_parameter('fov_start_angle').value)
        self.fov_end = math.radians(self.get_parameter('fov_end_angle').value)
        self.min_obstacle_points = self.get_parameter('min_obstacle_points').value
        self.consecutive_detections = self.get_parameter('consecutive_detections').value
        self.account_rplidar_transform = self.get_parameter('account_rplidar_transform').value
        exclude_ranges = self.get_parameter('exclude_angle_ranges').value
        scan_topic = self.get_parameter('scan_topic').value
        
        # Convert exclude ranges from degrees to radians
        self.exclude_ranges = []
        if exclude_ranges:
            for i in range(0, len(exclude_ranges), 2):
                if i + 1 < len(exclude_ranges):
                    start_deg = exclude_ranges[i]
                    end_deg = exclude_ranges[i + 1]
                    self.exclude_ranges.append((
                        math.radians(start_deg),
                        math.radians(end_deg)
                    ))
        
        # Publisher for obstacle detection
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detector', 10)
        
        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        # History for consecutive detection filtering
        self.detection_history = deque(maxlen=self.consecutive_detections)
        
        self.get_logger().info(f'Obstacle Avoidance Node Started')
        self.get_logger().info(f'FOV: {math.degrees(self.fov_start):.1f}° to {math.degrees(self.fov_end):.1f}°')
        self.get_logger().info(f'Min Distance Threshold: {self.min_distance:.2f}m')
        self.get_logger().info(f'Min Obstacle Points: {self.min_obstacle_points}')
        self.get_logger().info(f'Consecutive Detections Required: {self.consecutive_detections}')
        self.get_logger().info(f'Account RPLIDAR Transform: {self.account_rplidar_transform}')
        if self.exclude_ranges:
            exclude_str = ', '.join([f'{math.degrees(r[0]):.1f}°-{math.degrees(r[1]):.1f}°' for r in self.exclude_ranges])
            self.get_logger().info(f'Excluded Angle Ranges: {exclude_str}')
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def is_in_excluded_range(self, angle):
        """Check if angle is in an excluded range (e.g., sensor mount)"""
        for exclude_start, exclude_end in self.exclude_ranges:
            if exclude_start <= exclude_end:
                # Normal range
                if exclude_start <= angle <= exclude_end:
                    return True
            else:
                # Wrapping range
                if angle >= exclude_start or angle <= exclude_end:
                    return True
        return False
    
    def is_in_fov(self, angle):
        """Check if angle is within the defined FOV"""
        # RPLIDAR transforms angles: angle_published = M_PI - angle_original
        # So we need to account for this transformation
        if self.account_rplidar_transform:
            # Reverse the RPLIDAR transformation: original = M_PI - published
            angle = math.pi - angle
        
        # Normalize angle to [-pi, pi]
        angle = self.normalize_angle(angle)
        
        # Convert to [0, 2*pi] for easier comparison
        if angle < 0:
            angle += 2 * math.pi
        
        # Check if in excluded range first
        if self.is_in_excluded_range(angle):
            return False
        
        # Handle FOV that might wrap around 0/360 degrees
        if self.fov_start <= self.fov_end:
            # Normal case: FOV doesn't wrap
            return self.fov_start <= angle <= self.fov_end
        else:
            # Wrapping case: FOV crosses 0 degrees
            return angle >= self.fov_start or angle <= self.fov_end
    
    def scan_callback(self, msg):
        """Process laser scan data and detect obstacles"""
        obstacle_points = []  # Store all obstacle points in FOV
        min_detected_distance = float('inf')
        detection_angle = None
        total_points_in_fov = 0
        points_below_threshold = 0
        
        # Edge case: Check if scan message is valid
        if not msg.ranges or len(msg.ranges) == 0:
            if not hasattr(self, '_empty_scan_warned'):
                self.get_logger().warn('Received empty scan message')
                self._empty_scan_warned = True
            # Publish False for empty scans
            obstacle_msg = Bool()
            obstacle_msg.data = False
            self.obstacle_pub.publish(obstacle_msg)
            return
        
        # Iterate through all scan points
        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings (NaN, Inf)
            if math.isnan(distance) or math.isinf(distance):
                continue
            
            # Skip readings outside valid range
            if distance < msg.range_min or distance > msg.range_max:
                continue
            
            # Edge case: Skip readings that are too close (likely noise or sensor error)
            if distance < 0.05:  # 5cm minimum - likely noise
                continue
            
            # Calculate angle for this reading (this is the angle as published by RPLIDAR)
            angle = msg.angle_min + i * msg.angle_increment
            
            # Check if this point is within our FOV
            if self.is_in_fov(angle):
                total_points_in_fov += 1
                # Check if obstacle is within threshold
                if distance < self.min_distance:
                    points_below_threshold += 1
                    obstacle_points.append((distance, angle))
                    if distance < min_detected_distance:
                        min_detected_distance = distance
                        detection_angle = angle
        
        # Require minimum number of points to confirm obstacle (noise filtering)
        obstacle_detected = len(obstacle_points) >= self.min_obstacle_points
        
        # Add to detection history
        self.detection_history.append(obstacle_detected)
        
        # Require consecutive detections to avoid false positives
        if len(self.detection_history) >= self.consecutive_detections:
            # Only confirm if all recent detections agree
            confirmed_obstacle = all(self.detection_history)
        else:
            # Not enough history yet, use current detection
            confirmed_obstacle = obstacle_detected
        
        # Publish obstacle detection status - ALWAYS publish, even if False
        obstacle_msg = Bool()
        obstacle_msg.data = confirmed_obstacle
        self.obstacle_pub.publish(obstacle_msg)
        
        # Debug logging (first few scans to understand what's happening)
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        
        if self._debug_counter <= 5:
            self.get_logger().info(
                f'DEBUG Scan #{self._debug_counter}: '
                f'Total points in FOV: {total_points_in_fov}, '
                f'Points below threshold ({self.min_distance}m): {points_below_threshold}, '
                f'Obstacle points: {len(obstacle_points)}, '
                f'Required: {self.min_obstacle_points}, '
                f'Detected: {obstacle_detected}, '
                f'Confirmed: {confirmed_obstacle}'
            )
        
        # Log detection (only when obstacle is detected to avoid spam)
        if confirmed_obstacle:
            angle_deg = math.degrees(detection_angle) if detection_angle is not None else 0.0
            self.get_logger().warn(
                f'OBSTACLE DETECTED! Distance: {min_detected_distance:.2f}m at {angle_deg:.1f}° '
                f'({len(obstacle_points)}/{self.min_obstacle_points} points, '
                f'{total_points_in_fov} total in FOV)'
            )
        else:
            # Log clear status occasionally
            if not hasattr(self, 'clear_counter'):
                self.clear_counter = 0
            self.clear_counter += 1
            if self.clear_counter >= 50:  # Log every 50 scans when clear
                self.get_logger().info(
                    f'Path clear - No obstacles detected '
                    f'({total_points_in_fov} points in FOV, '
                    f'{points_below_threshold} below threshold)'
                )
                self.clear_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  