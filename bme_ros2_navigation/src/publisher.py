#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class SimpleCmdVelToJoints(Node):
    def __init__(self):
        super().__init__('simple_cmd_vel_to_joints')

        # Subscribe to teleop input
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # Avoid unused warning

        # Publish to the joint velocity controller
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # Create joint velocity message
        joint_velocity_msg = Float64MultiArray()

        # Simple mapping: forward sets all joints same direction
        if msg.linear.x > 0:
            joint_velocity_msg.data = [2.0, 2.0, 2.0]
        elif msg.linear.x < 0:
            joint_velocity_msg.data = [-2.0, -2.0, -2.0]
        elif msg.angular.z > 0:
            joint_velocity_msg.data = [1.0, 0.5, -1.0]  # turn right
        elif msg.angular.z < 0:
            joint_velocity_msg.data = [-1.0, -0.5, 1.0]  # turn left
        else:
            joint_velocity_msg.data = [0.0, 0.0, 0.0]  # stop

        # Publish to controller
        self.publisher_.publish(joint_velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCmdVelToJoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
