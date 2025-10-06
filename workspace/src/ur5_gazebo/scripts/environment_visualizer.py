#!/usr/bin/env python3
"""
Publishes visualization markers for the environment objects (blocks, bins, etc.)
so they can be seen in Foxglove when running headless simulation.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class EnvironmentVisualizer(Node):
    def __init__(self):
        super().__init__('environment_visualizer')
        
        self.marker_pub = self.create_publisher(MarkerArray, '/environment_markers', 10)
        
        # Publish markers at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('Environment visualizer started')
    
    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Red block marker
        block_marker = Marker()
        block_marker.header.frame_id = "world"
        block_marker.header.stamp = self.get_clock().now().to_msg()
        block_marker.ns = "environment"
        block_marker.id = 0
        block_marker.type = Marker.CUBE
        block_marker.action = Marker.ADD
        block_marker.pose.position.x = 0.5
        block_marker.pose.position.y = 0.3
        block_marker.pose.position.z = 0.05
        block_marker.pose.orientation.w = 1.0
        block_marker.scale.x = 0.05
        block_marker.scale.y = 0.05
        block_marker.scale.z = 0.1
        block_marker.color.r = 1.0
        block_marker.color.g = 0.0
        block_marker.color.b = 0.0
        block_marker.color.a = 1.0
        marker_array.markers.append(block_marker)
        
        # Gray bin marker
        bin_marker = Marker()
        bin_marker.header.frame_id = "world"
        bin_marker.header.stamp = self.get_clock().now().to_msg()
        bin_marker.ns = "environment"
        bin_marker.id = 1
        bin_marker.type = Marker.CUBE
        bin_marker.action = Marker.ADD
        bin_marker.pose.position.x = 0.7
        bin_marker.pose.position.y = 0.0
        bin_marker.pose.position.z = 0.05
        bin_marker.pose.orientation.w = 1.0
        bin_marker.scale.x = 0.2
        bin_marker.scale.y = 0.2
        bin_marker.scale.z = 0.1
        bin_marker.color.r = 0.5
        bin_marker.color.g = 0.5
        bin_marker.color.b = 0.5
        bin_marker.color.a = 1.0
        marker_array.markers.append(bin_marker)
        
        # Ground plane (for reference)
        ground_marker = Marker()
        ground_marker.header.frame_id = "world"
        ground_marker.header.stamp = self.get_clock().now().to_msg()
        ground_marker.ns = "environment"
        ground_marker.id = 2
        ground_marker.type = Marker.CUBE
        ground_marker.action = Marker.ADD
        ground_marker.pose.position.x = 0.0
        ground_marker.pose.position.y = 0.0
        ground_marker.pose.position.z = -0.01
        ground_marker.pose.orientation.w = 1.0
        ground_marker.scale.x = 3.0
        ground_marker.scale.y = 3.0
        ground_marker.scale.z = 0.02
        ground_marker.color.r = 0.3
        ground_marker.color.g = 0.3
        ground_marker.color.b = 0.3
        ground_marker.color.a = 0.8
        marker_array.markers.append(ground_marker)
        
        # Camera visualization (small box)
        camera_marker = Marker()
        camera_marker.header.frame_id = "world"
        camera_marker.header.stamp = self.get_clock().now().to_msg()
        camera_marker.ns = "environment"
        camera_marker.id = 3
        camera_marker.type = Marker.CUBE
        camera_marker.action = Marker.ADD
        camera_marker.pose.position.x = 0.5
        camera_marker.pose.position.y = 0.0
        camera_marker.pose.position.z = 1.0
        camera_marker.pose.orientation.w = 1.0
        camera_marker.scale.x = 0.05
        camera_marker.scale.y = 0.05
        camera_marker.scale.z = 0.05
        camera_marker.color.r = 0.0
        camera_marker.color.g = 1.0
        camera_marker.color.b = 1.0
        camera_marker.color.a = 1.0
        marker_array.markers.append(camera_marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

