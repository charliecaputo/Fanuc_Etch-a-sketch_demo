#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    import smbus2
except ImportError:
    smbus2 = None

TCA9548A_ADDR = 0x70
AS5600_ADDR = 0x36
ANGLE_MSB = 0x0E


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


class AS5600MuxNode(Node):
    def __init__(self):
        super().__init__('as5600_x_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('mux_address', TCA9548A_ADDR)
        self.declare_parameter('sensor_address', AS5600_ADDR)
        self.declare_parameter('x_channel', 0)
        self.declare_parameter('publish_hz', 30.0)

        self.declare_parameter('x_zero_deg', 0.0)
        self.declare_parameter('x_scale_mm_per_deg', 1.0)
        self.declare_parameter('x_limit_min_mm', -100.0)
        self.declare_parameter('x_limit_max_mm', 100.0)
        

        if smbus2 is None:
            raise RuntimeError(
                'smbus2 is required. Install with: pip install smbus2'
            )

        self.bus_num = int(self.get_parameter('i2c_bus').value)
        self.mux_addr = int(self.get_parameter('mux_address').value)
        self.sensor_addr = int(self.get_parameter('sensor_address').value)

        self.x_channel = int(self.get_parameter('x_channel').value)

        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.x_zero_deg = float(self.get_parameter('x_zero_deg').value)
        self.x_scale = float(self.get_parameter('x_scale_mm_per_deg').value)
        self.x_min = float(self.get_parameter('x_limit_min_mm').value)
        self.x_max = float(self.get_parameter('x_limit_max_mm').value)

        self.bus = smbus2.SMBus(self.bus_num)

        self.publisher = self.create_publisher(
            Float32MultiArray,
            'encoder_x_mm',
            10
        )

        period = 1.0 / self.publish_hz

        self.timer = self.create_timer(
            period,
            self.timer_callback
        )

        self.get_logger().info('AS5600 X-axis node started')

    def select_channel(self, channel):
        self.bus.write_byte(self.mux_addr, 1 << channel)
        time.sleep(0.001)

    def read_angle_deg(self):
        self.select_channel(self.x_channel)

        raw = self.bus.read_i2c_block_data(
            self.sensor_addr,
            ANGLE_MSB,
            2
        )

        angle_raw = ((raw[0] << 8) | raw[1]) & 0x0FFF

        return (angle_raw / 4096.0) * 360.0

    def angle_to_mm(self, angle_deg):
        delta = angle_deg - self.x_zero_deg
        mm = delta * self.x_scale
        return clamp(mm, self.x_min, self.x_max)

    def timer_callback(self):
        try:
            x_deg = self.read_angle_deg()
            x_mm = self.angle_to_mm(x_deg)

            msg = Float32MultiArray()
            msg.data = [float(x_deg), float(x_mm)]

            self.publisher.publish(msg)
            
            #self.get_logger().info(
            #    f'Angle: {x_deg:.2f} deg, Position: {x_mm:.2f} mm'
            #)
            
        except Exception as exc:
            self.get_logger().error(f'I2C read failed: {exc}')


def main():
    rclpy.init()

    node = AS5600MuxNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
