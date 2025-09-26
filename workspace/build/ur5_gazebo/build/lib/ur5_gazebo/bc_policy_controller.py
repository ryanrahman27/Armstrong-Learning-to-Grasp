#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
import tensorflow as tf
from tensorflow import keras
import pickle
import os


class BCPolicyController(Node):
    def __init__(self):
        super().__init__('bc_policy_controller')
        
        # Declare parameters
        self.declare_parameter('model_path', '/workspace/bc_model.h5')
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        update_rate = self.get_parameter('update_rate').value
        
        # Load model and normalization parameters
        self.load_model(model_path)
        
        # State variables
        self.current_joint_state = None
        self.block_pose = None
        self.policy_active = False
        
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
        
        self.policy_control_sub = self.create_subscription(
            Bool,
            '/bc_policy_active',
            self.policy_control_callback,
            10
        )
        
        # Publishers
        self.joint_command_pub = self.create_publisher(
            Float32MultiArray,
            '/joint_position_commands',
            10
        )
        
        self.policy_status_pub = self.create_publisher(
            Bool,
            '/bc_policy_status',
            10
        )
        
        # Timer for policy execution
        self.policy_timer = self.create_timer(
            1.0 / update_rate,
            self.policy_update_callback
        )
        
        self.get_logger().info('BC Policy Controller initialized')
        self.get_logger().info(f'Model loaded from: {model_path}')

    def load_model(self, model_path):
        """Load trained BC model and normalization parameters"""
        try:
            # Load model
            self.model = keras.models.load_model(model_path)
            self.get_logger().info('BC model loaded successfully')
            
            # Load normalization parameters
            norm_path = model_path.replace('.h5', '_norm.pkl')
            with open(norm_path, 'rb') as f:
                norm_params = pickle.load(f)
            
            self.input_mean = norm_params['input_mean']
            self.input_std = norm_params['input_std']
            self.output_mean = norm_params['output_mean']
            self.output_std = norm_params['output_std']
            
            self.get_logger().info('Normalization parameters loaded')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg

    def block_pose_callback(self, msg):
        """Store current block pose"""
        self.block_pose = msg

    def policy_control_callback(self, msg):
        """Handle policy activation/deactivation"""
        self.policy_active = msg.data
        
        if self.policy_active:
            self.get_logger().info('BC Policy activated')
        else:
            self.get_logger().info('BC Policy deactivated')
        
        # Publish status
        status_msg = Bool()
        status_msg.data = self.policy_active
        self.policy_status_pub.publish(status_msg)

    def policy_update_callback(self):
        """Execute policy if active"""
        if not self.policy_active:
            return
        
        if self.current_joint_state is None or self.block_pose is None:
            self.get_logger().warn('Missing joint state or block pose for policy execution')
            return
        
        try:
            # Prepare input
            input_state = self.prepare_input()
            
            # Get policy prediction
            predicted_action = self.predict_action(input_state)
            
            # Publish joint commands
            self.publish_joint_commands(predicted_action)
            
        except Exception as e:
            self.get_logger().error(f'Policy execution failed: {e}')

    def prepare_input(self):
        """Prepare input state for the policy"""
        # Current joint positions
        joint_positions = list(self.current_joint_state.position)
        
        # Block position and orientation
        block_position = [
            self.block_pose.pose.position.x,
            self.block_pose.pose.position.y,
            self.block_pose.pose.position.z
        ]
        
        block_orientation = [
            self.block_pose.pose.orientation.x,
            self.block_pose.pose.orientation.y,
            self.block_pose.pose.orientation.z,
            self.block_pose.pose.orientation.w
        ]
        
        # Combine input features
        input_state = joint_positions + block_position + block_orientation
        
        return np.array(input_state)

    def predict_action(self, input_state):
        """Use BC model to predict next action"""
        # Normalize input
        input_normalized = (input_state - self.input_mean) / self.input_std
        
        # Reshape for model input
        input_batch = input_normalized.reshape(1, -1)
        
        # Predict
        output_normalized = self.model.predict(input_batch, verbose=0)
        
        # Denormalize output
        output_denormalized = output_normalized * self.output_std + self.output_mean
        
        return output_denormalized[0]  # Remove batch dimension

    def publish_joint_commands(self, joint_positions):
        """Publish joint position commands"""
        msg = Float32MultiArray()
        msg.data = joint_positions.tolist()
        self.joint_command_pub.publish(msg)

    def get_success_metrics(self):
        """Calculate success metrics for comparison"""
        # This would be implemented based on specific success criteria
        # For example: distance to target, completion time, smoothness, etc.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BCPolicyController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()