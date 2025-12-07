#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class ObstacleAvoidanceNodeVisual(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Declare parameters with defaults
        self.declare_parameter('min_distance_threshold', 1.0)
        self.declare_parameter('fov_start_angle', 0.0)
        self.declare_parameter('fov_end_angle', 120.0)
        self.declare_parameter('scan_topic', '/scan')
        
        # Get parameters
        self.min_distance = self.get_parameter('min_distance_threshold').value
        self.fov_start = math.radians(self.get_parameter('fov_start_angle').value)
        self.fov_end = math.radians(self.get_parameter('fov_end_angle').value)
        scan_topic = self.get_parameter('scan_topic').value
        
        # Publishers
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detector', 10)
        self.marker_pub = self.create_publisher(Marker, '/obstacle_zone', 10)
        self.warning_pub = self.create_publisher(Marker, '/obstacle_warning', 10)
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f'Obstacle Avoidance Node Started')
        self.get_logger().info(f'FOV: {math.degrees(self.fov_start):.1f}° to {math.degrees(self.fov_end):.1f}°')
        self.get_logger().info(f'Min Distance Threshold: {self.min_distance:.2f}m')
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def is_in_fov(self, angle):
        """Check if angle is within the defined FOV (0 to 120 degrees)"""
        angle = self.normalize_angle(angle)
        
        if angle < 0:
            angle += 2 * math.pi
            
        return self.fov_start <= angle <= self.fov_end
    
    def publish_fov_marker(self, frame_id, obstacle_detected):
        """Publish FOV visualization marker"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fov_zone"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Draw FOV arc
        marker.scale.x = 0.02  # Line width
        
        # Color based on obstacle detection
        if obstacle_detected:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)  # Red
        else:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)  # Green
        
        # Create arc points
        marker.points = []
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))  # Center
        
        num_points = 50
        for i in range(num_points + 1):
            angle = self.fov_start + (self.fov_end - self.fov_start) * i / num_points
            x = self.min_distance * math.cos(angle)
            y = self.min_distance * math.sin(angle)
            marker.points.append(Point(x=x, y=y, z=0.0))
        
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))  # Back to center
        
        self.marker_pub.publish(marker)
    
    def publish_warning_text(self, frame_id, obstacle_detected, distance=None):
        """Publish warning text marker"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "warning_text"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        
        marker.scale.z = 0.2  # Text size
        
        if obstacle_detected:
            marker.text = f"⚠ OBSTACLE! {distance:.2f}m" if distance else "⚠ OBSTACLE!"
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        else:
            marker.text = "✓ PATH CLEAR"
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        self.warning_pub.publish(marker)
    
    def scan_callback(self, msg):
        """Process laser scan data and detect obstacles"""
        obstacle_detected = False
        min_detected_distance = float('inf')
        detection_angle = None
        
        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue
            
            if distance < msg.range_min or distance > msg.range_max:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            
            if self.is_in_fov(angle):
                if distance < self.min_distance:
                    obstacle_detected = True
                    if distance < min_detected_distance:
                        min_detected_distance = distance
                        detection_angle = angle
        
        # Publish obstacle detection status
        obstacle_msg = Bool()
        obstacle_msg.data = obstacle_detected
        self.obstacle_pub.publish(obstacle_msg)
        
        # Publish visualization markers
        self.publish_fov_marker(msg.header.frame_id, obstacle_detected)
        self.publish_warning_text(
            msg.header.frame_id, 
            obstacle_detected, 
            min_detected_distance if obstacle_detected else None
        )
        
        # Log detection
        if obstacle_detected:
            angle_deg = math.degrees(detection_angle)
            self.get_logger().warn(
                f'OBSTACLE DETECTED! Distance: {min_detected_distance:.2f}m at {angle_deg:.1f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNodeVisual()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()