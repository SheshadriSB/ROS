#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Empty  
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from oakd_roi_detector_interfaces.srv import GetDetections

import math
import time


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
        dt = 0.02 if self.prev_time is None else current_time - self.prev_time
        dt = max(dt, 0.001)  # safety

        self.integral += error * dt
        if self.integral_limit is not None:
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

        #Params
        self.declare_parameter('forward_distance', 1.0)
        self.declare_parameter('lateral_distance', 1.0)
        self.declare_parameter('distance_threshold', 0.08)
        self.declare_parameter('lateral_tolerance', 0.15)
        self.declare_parameter('max_forward_vel', 0.4)
        self.declare_parameter('max_lateral_vel', 0.4)
        self.declare_parameter('kp_x', 1.0)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.1)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.08)

        self.forward_distance = self.get_parameter('forward_distance').value
        self.lateral_distance = self.get_parameter('lateral_distance').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.lateral_tolerance = self.get_parameter('lateral_tolerance').value
        self.max_forward_vel = self.get_parameter('max_forward_vel').value
        self.max_lateral_vel = self.get_parameter('max_lateral_vel').value

        self.pid_x = SimplePID(
            kp=self.get_parameter('kp_x').value,
            ki=self.get_parameter('ki_x').value,
            kd=self.get_parameter('kd_x').value,
            output_limit=self.max_forward_vel
        )
        self.pid_y = SimplePID(
            kp=self.get_parameter('kp_y').value,
            ki=self.get_parameter('ki_y').value,
            kd=self.get_parameter('kd_y').value,
            output_limit=self.max_lateral_vel
        )

        # State
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_set = False
        self.moving = False
        self.phase = 0  # 0 = lateral, 1 = forward

        self.current_x = 0.0
        self.current_y = 0.0

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service client
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.scroll_client = self.create_client(
            GetDetections,
            '/get_detections',
            callback_group=self.client_cb_group
        )

        # Control loop
        self.create_timer(0.02, self.control_loop)  # 50 Hz

        # Status logging
        self.create_timer(5.0, self.log_status)  

        
        self.create_timer(2.0, self.try_request_scroll_side)

        self.get_logger().info('Scroll Approach Controller started (using SERVICE client)')

    def try_request_scroll_side(self):
        """Called periodically or from state machine - requests detection when ready"""
        if self.moving or self.target_set:
            return  # already moving

        if not self.scroll_client.wait_for_service(timeout_sec=1.5):
            self.get_logger().warn("Scroll detection service not available yet...")
            return

        self.get_logger().info("Requesting real scroll side from camera...")

        request = GetDetections.Request()
        future = self.scroll_client.call_async(request)

        
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()

            if not response.success:
                self.get_logger().warn(f"Detection failed: {response.status}")
                return

            
            self.get_logger().debug(f"roi1: '{response.roi1_class}' | roi2: '{response.roi2_class}'")

            
            REAL_INDICATORS = ["real_scroll", "real", "scroll"]   

            left_real = any(ind in response.roi1_class.lower() for ind in REAL_INDICATORS)
            right_real = any(ind in response.roi2_class.lower() for ind in REAL_INDICATORS)

            if right_real and not left_real:
                is_real_on_right = True
            elif left_real and not right_real:
                is_real_on_right = False
            else:
                self.get_logger().warn(
                    f"Ambiguous detection: left='{response.roi1_class}', right='{response.roi2_class}'"
                )
                return

            side = "RIGHT" if is_real_on_right else "LEFT"
            self.get_logger().info(f"Real scroll on {side} (roi1: {response.roi1_class}, roi2: {response.roi2_class})")

            self.set_new_target(is_real_on_right)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def set_new_target(self, is_real_on_right: bool):
        self.target_x = self.forward_distance
        self.target_y = self.lateral_distance if is_real_on_right else -self.lateral_distance
        self.target_set = True
        self.moving = True
        self.phase = 0
        self.pid_x.reset()
        self.pid_y.reset()
        self.get_logger().info(f"New target set: x={self.target_x:.2f}, y={self.target_y:+.2f}")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def control_loop(self):
        if not self.target_set or not self.moving:
            self.publish_zero_vel()
            return

        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        distance = math.hypot(error_x, error_y)

        if distance < self.distance_threshold:
            self.get_logger().info(f"Target reached! Distance remaining: {distance:.3f} m")
            self.publish_zero_vel()
            self.target_set = False
            self.moving = False
            self.phase = 0
            self.pid_x.reset()
            self.pid_y.reset()
            return

        if self.phase == 0:  # Lateral phase
            vx = 0.0
            vy = self.pid_y.compute(error_y)

            if abs(error_y) < self.lateral_tolerance:
                self.phase = 1
                self.pid_x.reset()
                self.get_logger().info("Lateral phase complete → switching to forward")
        else:  # Forward phase
            vx = self.pid_x.compute(error_x)
            vy = 0.0

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        self.cmd_vel_pub.publish(twist)

    def publish_zero_vel(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def log_status(self):
        if self.target_set:
            phase_str = "Lateral" if self.phase == 0 else "Forward"
            error_x = self.target_x - self.current_x
            error_y = self.target_y - self.current_y
            distance = math.hypot(error_x, error_y)
            self.get_logger().info(
                f"[{phase_str}] Pos: ({self.current_x:+.3f}, {self.current_y:+.3f}) | "
                f"Err: ({error_x:+.3f}, {error_y:+.3f}) | Dist: {distance:.3f}m"
            )
        else:
            self.get_logger().info(
                f"Idle | Pos: ({self.current_x:+.3f}, {self.current_y:+.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScrollApproachController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
