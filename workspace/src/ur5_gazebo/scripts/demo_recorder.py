#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import rosbag2_py
from rclpy.serialization import serialize_message
import os
from datetime import datetime


class DemoRecorder(Node):
    def __init__(self):
        super().__init__('demo_recorder')
        
        # Recording state
        self.recording = False
        self.bag_writer = None
        self.current_bag_path = None
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.block_pose_sub = self.create_subscription(
            PoseStamped,
            '/block_pose',
            self.block_pose_callback,
            10
        )
        
        self.record_control_sub = self.create_subscription(
            Bool,
            '/record_demo',
            self.record_control_callback,
            10
        )
        
        # Publishers
        self.recording_status_pub = self.create_publisher(
            Bool,
            '/recording_status',
            10
        )
        
        # Create data directory
        self.data_dir = '/data/demonstrations'
        os.makedirs(self.data_dir, exist_ok=True)
        
        self.get_logger().info('Demo recorder initialized')
        self.get_logger().info('Send True to /record_demo to start recording')
        self.get_logger().info('Send False to /record_demo to stop recording')

    def record_control_callback(self, msg):
        """Handle recording start/stop commands"""
        if msg.data and not self.recording:
            self.start_recording()
        elif not msg.data and self.recording:
            self.stop_recording()

    def start_recording(self):
        """Start recording demonstration"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_bag_path = os.path.join(self.data_dir, f"demo_{timestamp}")
        
        # Create rosbag writer
        storage_options = rosbag2_py.StorageOptions(
            uri=self.current_bag_path,
            storage_id='sqlite3'
        )
        
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        self.bag_writer = rosbag2_py.SequentialWriter()
        self.bag_writer.open(storage_options, converter_options)
        
        # Create topics
        joint_state_topic = rosbag2_py.TopicMetadata(
            name='/joint_states',
            type='sensor_msgs/msg/JointState',
            serialization_format='cdr'
        )
        
        block_pose_topic = rosbag2_py.TopicMetadata(
            name='/block_pose',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr'
        )
        
        self.bag_writer.create_topic(joint_state_topic)
        self.bag_writer.create_topic(block_pose_topic)
        
        self.recording = True
        self.get_logger().info(f'Started recording to {self.current_bag_path}')
        
        # Publish recording status
        status_msg = Bool()
        status_msg.data = True
        self.recording_status_pub.publish(status_msg)

    def stop_recording(self):
        """Stop recording demonstration"""
        if self.bag_writer:
            self.bag_writer.close()
            self.bag_writer = None
        
        self.recording = False
        self.get_logger().info(f'Stopped recording. Saved to {self.current_bag_path}')
        
        # Publish recording status
        status_msg = Bool()
        status_msg.data = False
        self.recording_status_pub.publish(status_msg)

    def joint_state_callback(self, msg):
        """Record joint state if recording is active"""
        if self.recording and self.bag_writer:
            timestamp = self.get_clock().now().nanoseconds
            serialized_msg = serialize_message(msg)
            self.bag_writer.write('/joint_states', serialized_msg, timestamp)

    def block_pose_callback(self, msg):
        """Record block pose if recording is active"""
        if self.recording and self.bag_writer:
            timestamp = self.get_clock().now().nanoseconds
            serialized_msg = serialize_message(msg)
            self.bag_writer.write('/block_pose', serialized_msg, timestamp)


def main(args=None):
    rclpy.init(args=args)
    node = DemoRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.recording:
            node.stop_recording()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()