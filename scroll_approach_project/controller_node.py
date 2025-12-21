#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import math

class SimplePID:
    def __init__(self, kp, ki, kd, integral_limit=1.0, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error):
        current_time = time.time()
        if self.prev_time is None:
            dt = 0.02  # fallback for first call
        else:
            dt = current_time - self.prev_time
        if dt <= 0:
            dt = 0.02

        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.output_limit is not None:
            output = max(min(output, self.output_limit), -self.output_limit)

        self.prev_error = error
        self.prev_time = current_time
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


class ScrollApproachController(Node):
    def __init__(self):
        super().__init__('scroll_approach_controller')

        # Parameters
        self.declare_parameter('forward_distance', 1.0)
        self.declare_parameter('lateral_distance', 1.0)
        self.declare_parameter('distance_threshold', 0.08)
        self.declare_parameter('max_linear_vel', 0.4)
        self.declare_parameter('kp_x', 1.0)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.1)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.08)

        self.forward_distance = self.get_parameter('forward_distance').value
        self.lateral_distance = self.get_parameter('lateral_distance').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value

        # PID controllers
        self.pid_x = SimplePID(
            kp=self.get_parameter('kp_x').value,
            ki=self.get_parameter('ki_x').value,
            kd=self.get_parameter('kd_x').value,
            output_limit=self.max_linear_vel
        )
        self.pid_y = SimplePID(
            kp=self.get_parameter('kp_y').value,
            ki=self.get_parameter('ki_y').value,
            kd=self.get_parameter('kd_y').value,
            output_limit=self.max_linear_vel
        )

        # State
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_set = False
        self.moving = False
        self.current_x = 0.0
        self.current_y = 0.0

        # Subscribers
        self.create_subscription(Bool, '/real_scroll_side', self.boolean_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control timer @ 50 Hz
        self.create_timer(0.02, self.control_loop)

        # Status timer
        self.create_timer(1.0, self.log_status)

        self.get_logger().info('Scroll Approach Controller fully initialized.')

    def boolean_callback(self, msg: Bool):
        if self.moving:
            self.get_logger().warn('New detection ignored — already moving.')
            return

        side = "RIGHT" if msg.data else "LEFT"
        self.target_x = self.forward_distance
        self.target_y = self.lateral_distance if msg.data else -self.lateral_distance

        self.target_set = True
        self.moving = True

        # Reset PIDs for new target
        self.pid_x.reset()
        self.pid_y.reset()

        self.get_logger().info(f'New target accepted: {side}')
        self.get_logger().info(f'  → x = {self.target_x:.2f} m, y = {self.target_y:+.2f} m')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def control_loop(self):
        if not self.target_set or not self.moving:
            self.publish_zero_vel()
            return

        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        distance = math.sqrt(error_x**2 + error_y**2)

        if distance < self.distance_threshold:
            self.get_logger().info(f'Target reached! Distance: {distance:.3f} m')
            self.publish_zero_vel()
            self.target_set = False
            self.moving = False
            self.pid_x.reset()
            self.pid_y.reset()
            return

        vx = self.pid_x.compute(error_x)
        vy = self.pid_y.compute(error_y)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def publish_zero_vel(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def log_status(self):
        if self.target_set:
            error_x = self.target_x - self.current_x
            error_y = self.target_y - self.current_y
            distance = math.sqrt(error_x**2 + error_y**2)
            self.get_logger().info(
                f'Pos: ({self.current_x:+.3f}, {self.current_y:+.3f}) | '
                f'Target: ({self.target_x:+.3f}, {self.target_y:+.3f}) | '
                f'Dist: {distance:.3f} m'
            )
        else:
            self.get_logger().info(
                f'Pos: ({self.current_x:+.3f}, {self.current_y:+.3f}) | Idle'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScrollApproachController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()