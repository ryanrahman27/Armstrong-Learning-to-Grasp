#!/usr/bin/env python3
"""
Simple script to make the UR5 arm move by publishing joint commands
This is a simple demonstration that doesn't require MoveIt
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import time

class SimpleArmMover(Node):
    def __init__(self):
        super().__init__('simple_arm_mover')
        
        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/forward_position_controller/commands', 
            10
        )
        
        # Subscriber to check current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = None
        self.get_logger().info('Simple Arm Mover initialized')
        self.get_logger().info('Waiting for joint states...')
        
        # Wait a moment for joint states to come in
        time.sleep(2)
        
        # Start movement sequence
        self.timer = self.create_timer(0.1, self.movement_loop)
        self.movement_phase = 0
        self.phase_counter = 0
        
    def joint_state_callback(self, msg):
        """Store current joint positions"""
        self.current_joint_positions = list(msg.position)
        
    def movement_loop(self):
        """Simple cyclic movement pattern"""
        if self.current_joint_positions is None:
            self.get_logger().info('Still waiting for joint states...', throttle_duration_sec=2.0)
            return
            
        # Define movement phases (each lasts 50 iterations = 5 seconds)
        phase_duration = 50
        
        # Different target positions for each phase
        if self.movement_phase == 0:
            # Home position
            targets = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            if self.phase_counter == 0:
                self.get_logger().info('Phase 0: Moving to home position')
                
        elif self.movement_phase == 1:
            # Reach forward
            targets = [0.0, -0.5, -1.0, -1.57, 0.0, 0.0]
            if self.phase_counter == 0:
                self.get_logger().info('Phase 1: Reaching forward')
                
        elif self.movement_phase == 2:
            # Move to side
            targets = [1.57, -1.0, -1.0, -1.57, 0.0, 0.0]
            if self.phase_counter == 0:
                self.get_logger().info('Phase 2: Moving to side')
                
        elif self.movement_phase == 3:
            # Wave motion
            t = self.phase_counter / phase_duration
            wave = math.sin(t * 4 * math.pi) * 0.5
            targets = [wave, -1.57, 0.0, -1.57, 0.0, wave]
            if self.phase_counter == 0:
                self.get_logger().info('Phase 3: Waving')
        else:
            # Reset to phase 0
            self.movement_phase = 0
            self.phase_counter = 0
            return
        
        # Publish command
        msg = Float64MultiArray()
        msg.data = targets
        self.joint_cmd_pub.publish(msg)
        
        # Update phase counter
        self.phase_counter += 1
        if self.phase_counter >= phase_duration:
            self.phase_counter = 0
            self.movement_phase += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimpleArmMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

