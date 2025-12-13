#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class CmdVelToJoints(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joints')

        # Subscribe to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Publish to the forward_velocity_controller
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )

        # Robot parameters
        self.high_velocity = 8.0  # Reduced amplitude for smooth motion
        self.phase_shift = math.pi / 2
        self.current_motion = 'stop'
        self.oscillation_frequency = 0.4  # Lower frequency for smooth undulation
        self.prev_joint_velocities = [0.0, 0.0, 0.0, 0.0]  # For smoothing

        # Timer for publishing joint velocities
        self.timer = self.create_timer(0.01, self.publish_joint_velocities)

    def cmd_vel_callback(self, msg: Twist):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        self.get_logger().info(f'Received cmd_vel: linear_velocity={linear_velocity}, angular_velocity={angular_velocity}')

        if linear_velocity > 0:
            self.current_motion = 'forward'
        elif angular_velocity < 0:
            self.current_motion = 'left'
        elif angular_velocity > 0:
            self.current_motion = 'right'
        elif linear_velocity == 0 and angular_velocity == 0:
            self.current_motion = 'stop'

    def publish_joint_velocities(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9

        if self.current_motion == 'stop':
            joint_velocities = [0.0, 0.0, 0.0, 0.0]
        elif self.current_motion == 'forward':
            joint_velocities = [
                self.high_velocity * 0.5 * math.sin(2 * math.pi * self.oscillation_frequency * current_time),
                self.high_velocity * 0.75 * math.sin(2 * math.pi * self.oscillation_frequency * current_time + self.phase_shift),
                self.high_velocity * math.sin(2 * math.pi * self.oscillation_frequency * current_time + 2 * self.phase_shift),
                self.high_velocity * 0
            ]
        elif self.current_motion == 'left':
            tail_oscillation = -self.high_velocity * math.sin(2 * math.pi * self.oscillation_frequency * current_time)
            joint_velocities = [0.0, 0.0, tail_oscillation, 0.0]
        elif self.current_motion == 'right':
            tail_oscillation = self.high_velocity * math.sin(2 * math.pi * self.oscillation_frequency * current_time)
            joint_velocities = [0.0, 0.0, tail_oscillation, 0.0]

        # Apply exponential smoothing filter
        alpha = 0.1
        joint_velocities = [
            alpha * new + (1 - alpha) * prev
            for new, prev in zip(joint_velocities, self.prev_joint_velocities)
        ]
        self.prev_joint_velocities = joint_velocities

        # Publish velocities
        joint_velocity_msg = Float64MultiArray()
        joint_velocity_msg.data = joint_velocities
        self.publisher_.publish(joint_velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_to_joints = CmdVelToJoints()

    try:
        rclpy.spin(cmd_vel_to_joints)
    except KeyboardInterrupt:
        pass

    cmd_vel_to_joints.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
