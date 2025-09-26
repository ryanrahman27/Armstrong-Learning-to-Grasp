#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String
import numpy as np
import time
import json
import os
from datetime import datetime


class ComparisonEvaluator(Node):
    def __init__(self):
        super().__init__('comparison_evaluator')
        
        # Evaluation parameters
        self.declare_parameter('num_trials', 10)
        self.declare_parameter('success_threshold', 0.05)  # 5cm
        self.declare_parameter('timeout_seconds', 60.0)
        
        self.num_trials = self.get_parameter('num_trials').value
        self.success_threshold = self.get_parameter('success_threshold').value
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        
        # Evaluation state
        self.current_trial = 0
        self.current_method = None  # 'moveit' or 'bc'
        self.trial_start_time = None
        self.trial_results = []
        
        # State tracking
        self.block_pose = None
        self.bin_position = Point()
        self.bin_position.x = 0.7
        self.bin_position.y = 0.0
        self.bin_position.z = 0.1
        
        self.joint_state = None
        self.evaluation_active = False
        
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
        
        self.evaluation_control_sub = self.create_subscription(
            String,
            '/start_evaluation',
            self.evaluation_control_callback,
            10
        )
        
        # Publishers
        self.moveit_trigger_pub = self.create_publisher(
            Bool,
            '/start_moveit_pickplace',
            10
        )
        
        self.bc_trigger_pub = self.create_publisher(
            Bool,
            '/bc_policy_active',
            10
        )
        
        self.evaluation_status_pub = self.create_publisher(
            String,
            '/evaluation_status',
            10
        )
        
        # Timer for evaluation monitoring
        self.evaluation_timer = self.create_timer(1.0, self.evaluation_timer_callback)
        
        self.get_logger().info('Comparison Evaluator initialized')
        self.get_logger().info(f'Will run {self.num_trials} trials per method')
        self.get_logger().info('Send "moveit" or "bc" to /start_evaluation to begin')

    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.joint_state = msg

    def block_pose_callback(self, msg):
        """Store current block pose"""
        self.block_pose = msg

    def evaluation_control_callback(self, msg):
        """Handle evaluation start commands"""
        if not self.evaluation_active:
            if msg.data.lower() in ['moveit', 'bc']:
                self.start_evaluation(msg.data.lower())
            else:
                self.get_logger().error('Invalid method. Use "moveit" or "bc"')

    def start_evaluation(self, method):
        """Start evaluation for specified method"""
        self.current_method = method
        self.current_trial = 0
        self.trial_results = []
        self.evaluation_active = True
        
        self.get_logger().info(f'Starting evaluation for {method} method')
        self.start_trial()

    def start_trial(self):
        """Start a single trial"""
        if self.current_trial >= self.num_trials:
            self.finish_evaluation()
            return
        
        self.current_trial += 1
        self.trial_start_time = time.time()
        
        self.get_logger().info(f'Starting trial {self.current_trial}/{self.num_trials} for {self.current_method}')
        
        # Reset environment (in real scenario, would reset block position)
        time.sleep(1.0)
        
        # Trigger appropriate method
        if self.current_method == 'moveit':
            msg = Bool()
            msg.data = True
            self.moveit_trigger_pub.publish(msg)
        elif self.current_method == 'bc':
            msg = Bool()
            msg.data = True
            self.bc_trigger_pub.publish(msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'{self.current_method}_trial_{self.current_trial}'
        self.evaluation_status_pub.publish(status_msg)

    def evaluation_timer_callback(self):
        """Monitor evaluation progress"""
        if not self.evaluation_active or self.trial_start_time is None:
            return
        
        elapsed_time = time.time() - self.trial_start_time
        
        # Check for timeout
        if elapsed_time > self.timeout_seconds:
            self.record_trial_result(success=False, completion_time=elapsed_time, reason='timeout')
            self.stop_current_method()
            self.start_trial()
            return
        
        # Check for success
        if self.check_trial_success():
            self.record_trial_result(success=True, completion_time=elapsed_time, reason='success')
            self.stop_current_method()
            self.start_trial()

    def check_trial_success(self):
        """Check if current trial is successful"""
        if self.block_pose is None:
            return False
        
        # Calculate distance from block to bin
        block_pos = self.block_pose.pose.position
        distance = np.sqrt(
            (block_pos.x - self.bin_position.x) ** 2 +
            (block_pos.y - self.bin_position.y) ** 2 +
            (block_pos.z - self.bin_position.z) ** 2
        )
        
        return distance < self.success_threshold

    def stop_current_method(self):
        """Stop the currently running method"""
        if self.current_method == 'moveit':
            msg = Bool()
            msg.data = False
            self.moveit_trigger_pub.publish(msg)
        elif self.current_method == 'bc':
            msg = Bool()
            msg.data = False
            self.bc_trigger_pub.publish(msg)

    def record_trial_result(self, success, completion_time, reason):
        """Record result of current trial"""
        result = {
            'trial': self.current_trial,
            'method': self.current_method,
            'success': success,
            'completion_time': completion_time,
            'reason': reason,
            'timestamp': datetime.now().isoformat()
        }
        
        self.trial_results.append(result)
        
        self.get_logger().info(
            f'Trial {self.current_trial} result: {reason} '
            f'({"success" if success else "failed"}) in {completion_time:.2f}s'
        )

    def finish_evaluation(self):
        """Finish evaluation and calculate metrics"""
        self.evaluation_active = False
        
        # Calculate metrics
        successful_trials = [r for r in self.trial_results if r['success']]
        success_rate = len(successful_trials) / len(self.trial_results)
        
        completion_times = [r['completion_time'] for r in successful_trials]
        avg_completion_time = np.mean(completion_times) if completion_times else 0
        
        # Generate report
        report = {
            'method': self.current_method,
            'total_trials': len(self.trial_results),
            'successful_trials': len(successful_trials),
            'success_rate': success_rate,
            'average_completion_time': avg_completion_time,
            'detailed_results': self.trial_results
        }
        
        # Save report
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = f'/data/evaluation_{self.current_method}_{timestamp}.json'
        
        os.makedirs('/data', exist_ok=True)
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)
        
        # Log summary
        self.get_logger().info(f'Evaluation completed for {self.current_method}')
        self.get_logger().info(f'Success rate: {success_rate:.2%}')
        self.get_logger().info(f'Average completion time: {avg_completion_time:.2f}s')
        self.get_logger().info(f'Report saved to: {report_path}')
        
        # Publish final status
        status_msg = String()
        status_msg.data = f'{self.current_method}_completed'
        self.evaluation_status_pub.publish(status_msg)

    def compare_methods(self, moveit_report_path, bc_report_path):
        """Compare results between MoveIt and BC methods"""
        try:
            with open(moveit_report_path, 'r') as f:
                moveit_data = json.load(f)
            
            with open(bc_report_path, 'r') as f:
                bc_data = json.load(f)
            
            comparison = {
                'moveit_success_rate': moveit_data['success_rate'],
                'bc_success_rate': bc_data['success_rate'],
                'moveit_avg_time': moveit_data['average_completion_time'],
                'bc_avg_time': bc_data['average_completion_time'],
                'success_rate_difference': bc_data['success_rate'] - moveit_data['success_rate'],
                'time_difference': bc_data['average_completion_time'] - moveit_data['average_completion_time']
            }
            
            self.get_logger().info('=== COMPARISON RESULTS ===')
            self.get_logger().info(f"MoveIt Success Rate: {comparison['moveit_success_rate']:.2%}")
            self.get_logger().info(f"BC Success Rate: {comparison['bc_success_rate']:.2%}")
            self.get_logger().info(f"MoveIt Avg Time: {comparison['moveit_avg_time']:.2f}s")
            self.get_logger().info(f"BC Avg Time: {comparison['bc_avg_time']:.2f}s")
            
            return comparison
            
        except Exception as e:
            self.get_logger().error(f'Failed to compare methods: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ComparisonEvaluator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()