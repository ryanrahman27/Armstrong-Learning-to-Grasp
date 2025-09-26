#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs


class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')
        
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.block_pose_pub = self.create_publisher(
            PoseStamped,
            '/block_pose',
            10
        )
        
        self.detected_image_pub = self.create_publisher(
            Image,
            '/detected_blocks',
            10
        )
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.get_logger().info('Camera detector node started')

    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process camera image to detect colored blocks"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect red blocks
            block_pose = self.detect_red_block(cv_image)
            
            if block_pose is not None:
                # Transform to world frame
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'world', 'camera_link', rclpy.time.Time()
                    )
                    world_pose = tf2_geometry_msgs.do_transform_pose(block_pose, transform)
                    world_pose.header.stamp = self.get_clock().now().to_msg()
                    world_pose.header.frame_id = 'world'
                    
                    self.block_pose_pub.publish(world_pose)
                    self.get_logger().info(f'Block detected at: {world_pose.pose.position}')
                    
                except Exception as e:
                    self.get_logger().warn(f'Transform failed: {e}')
            
            # Publish annotated image
            annotated_image = self.annotate_image(cv_image, block_pose)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.detected_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def detect_red_block(self, image):
        """Detect red blocks using color segmentation"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 + mask2
        
        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate center point
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Create pose in camera frame
            if self.camera_matrix is not None:
                # Use camera intrinsics to get 3D position
                # Assuming block is on table at known height
                table_height = 0.05  # 5cm above ground
                
                # Back-project to 3D
                point_2d = np.array([[center_x, center_y]], dtype=np.float32)
                point_3d = cv2.undistortPoints(
                    point_2d.reshape(-1, 1, 2), 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # Calculate 3D position
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx = self.camera_matrix[0, 2]
                cy = self.camera_matrix[1, 2]
                
                # Simple depth estimation (assuming block is on table)
                depth = table_height
                x_3d = (point_3d[0, 0, 0] - cx) * depth / fx
                y_3d = (point_3d[0, 0, 1] - cy) * depth / fy
                z_3d = depth
                
                # Create pose message
                pose = PoseStamped()
                pose.header.frame_id = 'camera_link'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = x_3d
                pose.pose.position.y = y_3d
                pose.pose.position.z = z_3d
                pose.pose.orientation.w = 1.0
                
                return pose
        
        return None

    def annotate_image(self, image, block_pose):
        """Draw annotations on the image"""
        annotated = image.copy()
        
        if block_pose is not None:
            # Convert 3D pose back to 2D for visualization
            if self.camera_matrix is not None:
                point_3d = np.array([
                    [block_pose.pose.position.x],
                    [block_pose.pose.position.y],
                    [block_pose.pose.position.z]
                ])
                
                point_2d, _ = cv2.projectPoints(
                    point_3d, 
                    np.zeros(3), 
                    np.zeros(3), 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                x, y = int(point_2d[0, 0, 0]), int(point_2d[0, 0, 1])
                cv2.circle(annotated, (x, y), 10, (0, 255, 0), -1)
                cv2.putText(annotated, 'Block', (x+15, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return annotated


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()