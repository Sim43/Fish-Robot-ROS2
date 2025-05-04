#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('mogi_bot_odometry_node')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize joint positions and previous joint positions for filtering
        self.joint_positions = {'joint_1': 0.0, 'joint_2': 0.0, 'joint_3': 0.0, 'joint_4': 0.0}
        self.prev_joint_positions = {'joint_1': 0.0, 'joint_2': 0.0, 'joint_3': 0.0, 'joint_4': 0.0}

        # Initial robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Robot's orientation (heading)

        # Timer to publish data every 0.1 second (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Odometry Publisher Node has started')

    def joint_state_callback(self, msg):
        # Update joint positions using smoothing technique
        for i, name in enumerate(msg.name):
            if name in self.joint_positions:
                self.joint_positions[name] = msg.position[i]

    def timer_callback(self):
        # Calculate odometry based on joint positions
        self.calculate_odometry()
        self.publish_odometry()

    def calculate_odometry(self):
        # Get joint positions (assuming these are some form of encoder or joint angles)
        joint_1_pos = self.joint_positions['joint_1']
        joint_2_pos = self.joint_positions['joint_2']
        joint_3_pos = self.joint_positions['joint_3']
        joint_4_pos = self.joint_positions['joint_4']

        # Compute linear velocity (assume linear displacement per joint movement)
        # For simplicity, we assume that each joint corresponds to forward motion
        delta_joint_1 = joint_1_pos - self.prev_joint_positions['joint_1']
        delta_joint_2 = joint_2_pos - self.prev_joint_positions['joint_2']
        delta_joint_3 = joint_3_pos - self.prev_joint_positions['joint_3']
        delta_joint_4 = joint_4_pos - self.prev_joint_positions['joint_4']

        # Simple model: joint velocity causes robot's forward motion, rotation is negligible
        delta_x = (delta_joint_1 + delta_joint_2 + delta_joint_3 + delta_joint_4) * 0.1  # scale factor (adjust as needed)
        
        # Update position
        self.x += delta_x * math.cos(self.theta)
        self.y += delta_x * math.sin(self.theta)

        # Update orientation (theta) (for now assume the rotation is small and based on joint movements)
        self.theta += (delta_joint_1 + delta_joint_2 + delta_joint_3 + delta_joint_4) * 0.05  # small rotational factor

        # Update previous joint positions
        self.prev_joint_positions = self.joint_positions.copy()

    def publish_odometry(self):
        # Create odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        # Set position (updated x, y)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0  # Fixed z-position

        # Set orientation (calculated from theta)
        quat = self.euler_to_quaternion(self.theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Publish the message
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing Odometry: x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}')

        # Broadcast the transform from base_footprint to base_link
        self.broadcast_transform('base_footprint', 'base_link', self.x, self.y, 0.0, quat[0], quat[1], quat[2], quat[3])

        # Broadcast transforms for Camera, IMU, and Scan with the correct offsets
        self.broadcast_transform('base_link', 'camera_link', self.x - 0.01, self.y - 0.009, 0.10, quat[0], quat[1], quat[2], quat[3])
        self.broadcast_transform('base_link', 'imu_link', self.x, self.y, 0.0, quat[0], quat[1], quat[2], quat[3])
        self.broadcast_transform('base_link', 'scan_link', self.x - 0.01, self.y + 0.05, 0.17, quat[0], quat[1], quat[2], quat[3])

    def broadcast_transform(self, parent_frame, child_frame, x, y, z, quat_x, quat_y, quat_z, quat_w):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = quat_x
        transform.transform.rotation.y = quat_y
        transform.transform.rotation.z = quat_z
        transform.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, theta):
        # Convert Euler angle to quaternion (for 2D orientation)
        q = R.from_euler('z', theta).as_quat()
        return q

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
