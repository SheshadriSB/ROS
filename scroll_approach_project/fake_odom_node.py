#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_node')

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.last_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Publish odom at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_odom)

        self.get_logger().info('Fake odometry node started. Robot starts at (0,0).')

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 0:
            self.current_x += self.vx * dt
            self.current_y += self.vy * dt
            self.current_yaw += self.omega * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = 0.0

        # Simple yaw to quaternion
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.omega

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()