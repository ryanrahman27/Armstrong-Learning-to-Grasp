#!/usr/bin/env python3
"""
Simple pick and place demonstration using direct Gazebo commands
No MoveIt required - just moves joints directly!
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyJointEffort, GetModelState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class SimplePickPlace(Node):
    def __init__(self):
        super().__init__('simple_pick_place')
        
        # Service clients
        self.apply_effort_client = self.create_client(
            ApplyJointEffort,
            '/gazebo/apply_joint_effort'
        )
        
        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.current_joints = None
        self.get_logger().info('Simple Pick and Place initialized!')
        self.get_logger().info('Waiting for joint states...')
        
        # Wait for services
        while not self.apply_effort_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo services...')
        
        self.get_logger().info('Ready to start demonstration!')
        
    def joint_callback(self, msg):
        """Store current joint positions"""
        self.current_joints = dict(zip(msg.name, msg.position))
        
    def apply_joint_effort(self, joint_name, effort, duration_sec=2.0):
        """Apply effort to a joint"""
        request = ApplyJointEffort.Request()
        request.joint_name = joint_name
        request.effort = effort
        request.start_time.sec = 0
        request.start_time.nanosec = 0
        request.duration.sec = duration_sec
        request.duration.nanosec = 0
        
        future = self.apply_effort_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
        
    def run_demo(self):
        """Run the pick and place demonstration"""
        self.get_logger().info('')
        self.get_logger().info('🤖 ======================================')
        self.get_logger().info('🤖   SIMPLE PICK AND PLACE DEMO')
        self.get_logger().info('🤖 ======================================')
        self.get_logger().info('')
        
        # Wait for joint states
        rate = self.create_rate(10)
        while self.current_joints is None:
            self.get_logger().info('Waiting for joint states...', throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().info('✅ Joint states received!')
        time.sleep(2)
        
        # Sequence of movements
        movements = [
            ('Phase 1: Moving to home position', [
                ('shoulder_pan_joint', 20.0, 2.0),
                ('shoulder_lift_joint', -30.0, 2.0),
            ]),
            ('Phase 2: Reaching toward block', [
                ('shoulder_pan_joint', 15.0, 2.0),
                ('elbow_joint', -25.0, 2.0),
            ]),
            ('Phase 3: Lowering to grasp', [
                ('shoulder_lift_joint', 15.0, 1.5),
                ('wrist_1_joint', -10.0, 1.5),
            ]),
            ('Phase 4: Lifting block', [
                ('shoulder_lift_joint', -20.0, 2.0),
                ('elbow_joint', 15.0, 2.0),
            ]),
            ('Phase 5: Moving to bin', [
                ('shoulder_pan_joint', -25.0, 2.5),
                ('wrist_2_joint', 10.0, 2.0),
            ]),
            ('Phase 6: Placing block', [
                ('shoulder_lift_joint', 15.0, 2.0),
                ('wrist_1_joint', 10.0, 1.5),
            ]),
            ('Phase 7: Returning home', [
                ('shoulder_pan_joint', 0.0, 3.0),
                ('shoulder_lift_joint', 0.0, 3.0),
                ('elbow_joint', 0.0, 3.0),
            ]),
        ]
        
        for phase_name, joint_commands in movements:
            self.get_logger().info('')
            self.get_logger().info(f'🎯 {phase_name}')
            self.get_logger().info('')
            
            for joint_name, effort, duration in joint_commands:
                self.get_logger().info(f'  → Moving {joint_name}')
                success = self.apply_joint_effort(joint_name, effort, duration)
                if success:
                    self.get_logger().info(f'  ✅ {joint_name} command sent')
                else:
                    self.get_logger().warn(f'  ⚠️  {joint_name} command may have failed')
                time.sleep(duration + 0.5)
                
            time.sleep(1)  # Pause between phases
            
        self.get_logger().info('')
        self.get_logger().info('🎉 ======================================')
        self.get_logger().info('🎉   DEMONSTRATION COMPLETE!')
        self.get_logger().info('🎉 ======================================')
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    node = SimplePickPlace()
    
    try:
        # Run the demonstration
        node.run_demo()
        
        # Keep node alive for a bit
        time.sleep(5)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

