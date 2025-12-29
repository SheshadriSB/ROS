#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import struct
import serial

START_BYTE = 0xA5
TX_FRAME_SIZE = 14  # start + 12 (3 floats) + checksum

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        self.declare_parameter('port', '/dev/ttySTM32_MOTORS')  # motor MCU
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Motor serial port {port} opened at {baud} baud")
        except Exception as e:
            self.get_logger().fatal(f"Failed to open motor serial port: {e}")
            raise

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Motor bridge ready — TX only: vx, vy, omega → motor MCU')

    def cmd_vel_callback(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        omega = float(msg.angular.z)

        payload = struct.pack('<fff', vx, vy, omega)
        checksum = START_BYTE
        for b in payload:
            checksum ^= b & 0xFF

        frame = bytes([START_BYTE]) + payload + bytes([checksum & 0xFF])

        try:
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f"Motor serial write failed: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Motor serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()