#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint
)
from sensor_msgs.msg import JointState
import numpy as np
import time


class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')
        
        # MoveIt action client
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action'
        )
        
        # Subscribers
        self.block_pose_sub = self.create_subscription(
            PoseStamped,
            '/block_pose',
            self.block_pose_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # State variables
        self.current_joint_state = None
        self.block_pose = None
        self.pick_place_active = False
        
        # Predefined poses
        self.home_pose = [0, -1.57, 0, -1.57, 0, 0]  # Home position
        self.bin_pose = PoseStamped()
        self.bin_pose.header.frame_id = "world"
        self.bin_pose.pose.position.x = 0.7
        self.bin_pose.pose.position.y = 0.0
        self.bin_pose.pose.position.z = 0.15
        self.bin_pose.pose.orientation.w = 1.0
        
        self.get_logger().info('Pick and place controller initialized')

    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg

    def block_pose_callback(self, msg):
        """Handle block detection"""
        self.block_pose = msg
        
        if not self.pick_place_active:
            self.get_logger().info('Block detected, starting pick and place sequence')
            self.execute_pick_place_sequence()

    def execute_pick_place_sequence(self):
        """Execute complete pick and place sequence"""
        self.pick_place_active = True
        
        try:
            # 1. Move to home position
            self.get_logger().info('Moving to home position')
            self.move_to_joint_position(self.home_pose)
            time.sleep(2)
            
            # 2. Move to pick position (above block)
            if self.block_pose:
                pick_pose = self.calculate_pick_pose(self.block_pose)
                self.get_logger().info('Moving to pick position')
                self.move_to_pose(pick_pose)
                time.sleep(2)
                
                # 3. Lower to grasp
                grasp_pose = pick_pose
                grasp_pose.pose.position.z -= 0.05
                self.get_logger().info('Lowering to grasp')
                self.move_to_pose(grasp_pose)
                time.sleep(1)
                
                # 4. Simulate gripper close
                self.get_logger().info('Grasping block')
                time.sleep(1)
                
                # 5. Lift block
                lift_pose = grasp_pose
                lift_pose.pose.position.z += 0.1
                self.get_logger().info('Lifting block')
                self.move_to_pose(lift_pose)
                time.sleep(2)
                
                # 6. Move to bin position
                place_pose = self.bin_pose
                place_pose.pose.position.z += 0.1
                self.get_logger().info('Moving to bin')
                self.move_to_pose(place_pose)
                time.sleep(2)
                
                # 7. Lower to place
                place_pose.pose.position.z -= 0.05
                self.get_logger().info('Placing block')
                self.move_to_pose(place_pose)
                time.sleep(1)
                
                # 8. Release gripper
                self.get_logger().info('Releasing block')
                time.sleep(1)
                
                # 9. Return to home
                self.get_logger().info('Returning to home')
                self.move_to_joint_position(self.home_pose)
                
                self.get_logger().info('Pick and place sequence completed')
            
        except Exception as e:
            self.get_logger().error(f'Pick and place failed: {e}')
        
        finally:
            self.pick_place_active = False

    def calculate_pick_pose(self, block_pose):
        """Calculate pick pose above the block"""
        pick_pose = PoseStamped()
        pick_pose.header = block_pose.header
        pick_pose.pose.position.x = block_pose.pose.position.x
        pick_pose.pose.position.y = block_pose.pose.position.y
        pick_pose.pose.position.z = block_pose.pose.position.z + 0.15
        
        # Set downward orientation
        pick_pose.pose.orientation.x = 1.0
        pick_pose.pose.orientation.y = 0.0
        pick_pose.pose.orientation.z = 0.0
        pick_pose.pose.orientation.w = 0.0
        
        return pick_pose

    def move_to_pose(self, target_pose):
        """Move to target pose using MoveIt"""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False
        
        # Create motion plan request
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "manipulator"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Set target pose
        goal_msg.request.goal_constraints = [Constraints()]
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = "gripper_link"
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.constraint_region.primitive_poses = [target_pose.pose]
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = "gripper_link"
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        goal_msg.request.goal_constraints[0].position_constraints = [position_constraint]
        goal_msg.request.goal_constraints[0].orientation_constraints = [orientation_constraint]
        
        # Planning options
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
                return result_future.result() is not None
        
        return False

    def move_to_joint_position(self, joint_positions):
        """Move to joint position using MoveIt"""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False
        
        # Create motion plan request
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "manipulator"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Set joint constraints
        goal_msg.request.goal_constraints = [Constraints()]
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        for i, (name, position) in enumerate(zip(joint_names, joint_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_msg.request.goal_constraints[0].joint_constraints.append(joint_constraint)
        
        # Planning options
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
                return result_future.result() is not None
        
        return False


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()