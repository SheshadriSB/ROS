#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import struct
import serial
import time

START_BYTE = 0xA5
RX_FRAME_SIZE = 10  # start + 8 (2 floats) + checksum

class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')

        self.declare_parameter('port', '/dev/null')  # ← Change to your encoder MCU port
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Deadwheel serial port {port} opened at {baud} baud")
        except Exception as e:
            self.get_logger().fatal(f"Failed to open deadwheel serial port: {e}")
            raise

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(0.01, self.read_serial)  # 100 Hz

        # Integration state
        self.x = 0.0
        self.y = 0.0
        self.last_time = time.time()  # using time.time() for simplicity

        # RX buffer
        self.rx_buffer = bytearray()

        self.get_logger().info('Deadwheel bridge ready — RX only: omega_y, omega_x → odom')

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))

            while len(self.rx_buffer) >= RX_FRAME_SIZE:
                if self.rx_buffer[0] != START_BYTE:
                    self.rx_buffer.pop(0)
                    continue

                payload = bytes(self.rx_buffer[1:9])
                checksum_recv = self.rx_buffer[9]

                calc_checksum = START_BYTE
                for b in payload:
                    calc_checksum ^= b

                if calc_checksum != checksum_recv:
                    self.get_logger().debug("Checksum mismatch — dropping byte")
                    self.rx_buffer.pop(0)
                    continue

                omega_y, omega_x = struct.unpack('<ff', payload)

                current_time = time.time()
                dt = current_time - self.last_time
                if dt > 0.1 or dt <= 0:
                    dt = 0.01
                self.last_time = current_time

                self.x += omega_x * dt
                self.y += omega_y * dt

                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'

                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation.w = 1.0  # yaw = 0 (no angular feedback)

                odom.twist.twist.linear.x = omega_x
                odom.twist.twist.linear.y = omega_y

                self.odom_pub.publish(odom)

                del self.rx_buffer[:RX_FRAME_SIZE]

        except Exception as e:
            self.get_logger().error(f"Deadwheel serial read error: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Deadwheel serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()