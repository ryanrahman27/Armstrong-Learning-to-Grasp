#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import math

class SimpleJointController(Node):
    def __init__(self):
        super().__init__('simple_joint_controller')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.block_sub = self.create_subscription(PoseStamped, '/block_pose', self.move_to_block, 10)
        
    def move_to_block(self, msg):
        # Simple IK: point arm toward block
        block_x = msg.pose.position.x
        block_y = msg.pose.position.y
        angle = math.atan2(block_y, block_x)
        
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['base_to_arm']
        js.position = [angle]
        
        self.joint_pub.publish(js)
        self.get_logger().info(f'Moving arm to angle: {angle:.2f} rad')

def main():
    rclpy.init()
    controller = SimpleJointController()
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
