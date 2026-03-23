#!/usr/bin/env python3

import math
import carla
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from carla_interface import carla_utils


class Speedometer(Node):
    def __init__(self):
        super().__init__('speedometer')

        self.server_host = self.declare_parameter('server_connection.host', 'localhost').get_parameter_value().string_value
        self.server_port = self.declare_parameter('server_connection.port', 2000).get_parameter_value().integer_value
        self.timeout = self.declare_parameter('server_connection.timeout', 10.0).get_parameter_value().double_value
        self.publish_rate_hz = self.declare_parameter('speedometer.publish_rate_hz', 20.0).get_parameter_value().double_value

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn('Parameter speedometer.publish_rate_hz must be > 0.0. Falling back to 20.0 Hz.')
            self.publish_rate_hz = 20.0

        self.speed_publisher = self.create_publisher(Float32, '/carla/hero/speedometer', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_speed)

        self.client = carla.Client(self.server_host, self.server_port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()

        self.hero_vehicle = None

    def publish_speed(self):
        if self.hero_vehicle is None:
            self.hero_vehicle = carla_utils.get_hero_vehicle(self.world, self.timeout)
            if self.hero_vehicle is None:
                self.get_logger().error('Hero vehicle not found; speedometer will not be created.')
                return

        velocity = self.hero_vehicle.get_velocity()
        
        # Convert velocity vector to speed in m/s
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

        msg = Float32()
        msg.data = float(speed)
        self.speed_publisher.publish(msg)


def main():
    rclpy.init()

    node = Speedometer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()