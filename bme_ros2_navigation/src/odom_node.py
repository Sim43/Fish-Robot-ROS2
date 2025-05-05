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
        self.joint_positions = {
            'joint_1': 0.0,
            'joint_2': 0.0,
            'joint_3': 0.0,
            'joint_4': 0.0  # Thruster joint
        }
        self.prev_joint_positions = self.joint_positions.copy()

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
        self.joint_positions = {
            'joint_1': 0.0,
            'joint_2': 0.0,
            'joint_3': 0.0,
            'joint_4': 0.0  # Thruster joint
        }
        self.prev_joint_positions = self.joint_positions.copy()

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
        self.calculate_odometry()
        self.publish_odometry()

    def calculate_odometry(self):
        # Get current and delta positions
        deltas = {}
        for joint in self.joint_positions:
            deltas[joint] = self.joint_positions[joint] - self.prev_joint_positions[joint]

        # Assume thruster (joint_4) contributes more to forward motion
        delta_x = (
            deltas['joint_1'] +
            deltas['joint_2'] +
            deltas['joint_3'] +
            deltas['joint_4'] * 3.0  # Thruster has more impact
        ) * 0.1  # scale factor

        # Update position
        self.x += delta_x * math.cos(self.theta)
        self.y += delta_x * math.sin(self.theta)

        # Slight rotation update from tail flapping (optional)
        self.theta += (deltas['joint_1'] - deltas['joint_3']) * 0.02

        # Save for next cycle
        self.prev_joint_positions = self.joint_positions.copy()

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        quat = self.euler_to_quaternion(self.theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        self.publisher_.publish(msg)

        self.broadcast_transform('base_footprint', 'base_link', self.x, self.y, 0.0, *quat)
        self.broadcast_transform('base_link', 'camera_link', self.x - 0.01, self.y - 0.009, 0.10, *quat)
        self.broadcast_transform('base_link', 'imu_link', self.x, self.y, 0.0, *quat)
        self.broadcast_transform('base_link', 'scan_link', self.x - 0.01, self.y + 0.05, 0.17, *quat)

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

    def timer_callback(self):
        self.calculate_odometry()
        self.publish_odometry()

    def calculate_odometry(self):
        # Get current and delta positions
        deltas = {}
        for joint in self.joint_positions:
            deltas[joint] = self.joint_positions[joint] - self.prev_joint_positions[joint]

        # Assume thruster (joint_4) contributes more to forward motion
        delta_x = (
            deltas['joint_1'] +
            deltas['joint_2'] +
            deltas['joint_3'] +
            deltas['joint_4'] * 5.0  # Thruster has more impact
        ) * 0.1  # scale factor

        # Update position
        self.x += delta_x * math.cos(self.theta)
        self.y += delta_x * math.sin(self.theta)

        # Slight rotation update from tail flapping (optional)
        self.theta += (deltas['joint_1'] - deltas['joint_3']) * 0.02

        # Save for next cycle
        self.prev_joint_positions = self.joint_positions.copy()

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        quat = self.euler_to_quaternion(self.theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        self.publisher_.publish(msg)

        self.broadcast_transform('base_footprint', 'base_link', self.x, self.y, 0.0, *quat)

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
