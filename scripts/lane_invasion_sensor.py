#!/usr/bin/env python3

import carla
import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaLaneInvasionEvent
from carla_interface import carla_utils


class LaneInvasionSensor(Node):
    def __init__(self):
        super().__init__('lane_invasion_sensor')

        self.server_host = self.declare_parameter('server_connection.host', 'localhost').get_parameter_value().string_value
        self.server_port = self.declare_parameter('server_connection.port', 2000).get_parameter_value().integer_value
        self.timeout = self.declare_parameter('server_connection.timeout', 10.0).get_parameter_value().double_value

        self.publisher = self.create_publisher(CarlaLaneInvasionEvent, '/carla/hero/lane_invasion', 10)

        self.client = carla.Client(self.server_host, self.server_port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()

        self.hero_vehicle = None
        self.lane_invasion_sensor = None
        
        self.setup_lane_invasion_sensor()
        
    def setup_lane_invasion_sensor(self):
        # Find the hero vehicle
        if self.hero_vehicle is None:
            self.hero_vehicle = carla_utils.get_hero_vehicle(self.world, self.timeout)
            if self.hero_vehicle is None:
                self.get_logger().error('Hero vehicle not found; lane invasion sensor will not be created.')
                return

        # Set up the lane invasion sensor
        blueprint_library = self.world.get_blueprint_library()
        lane_invasion_bp = blueprint_library.find('sensor.other.lane_invasion')

        # Spawn the sensor and attach it to the hero vehicle
        spawn_point = carla.Transform(carla.Location(0, 0, 0))
        self.lane_invasion_sensor = self.world.spawn_actor(lane_invasion_bp, spawn_point, attach_to=self.hero_vehicle)
        self.get_logger().info('Spawning sensor: {}'.format(self.lane_invasion_sensor.type_id))

        def on_lane_invasion(event):
            msg = CarlaLaneInvasionEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            for marking in event.crossed_lane_markings:
                msg.crossed_lane_markings.append(marking.type)
            self.publisher.publish(msg)

        self.lane_invasion_sensor.listen(on_lane_invasion)
        
    def destroy_node(self):
        if self.lane_invasion_sensor is not None:
            if self.lane_invasion_sensor.is_listening:
                self.lane_invasion_sensor.stop()
            if self.lane_invasion_sensor.is_alive:
                self.lane_invasion_sensor.destroy()
            self.lane_invasion_sensor = None

        super().destroy_node()

def main():
    rclpy.init()

    node = LaneInvasionSensor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()