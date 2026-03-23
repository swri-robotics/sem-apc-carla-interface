#!/usr/bin/env python3

import weakref

import carla
import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaCollisionEvent
from carla_interface import carla_utils


class CollisionSensor(Node):
	def __init__(self):
		super().__init__('collision_sensor')

		self.server_host = self.declare_parameter('server_connection.host', 'localhost').get_parameter_value().string_value
		self.server_port = self.declare_parameter('server_connection.port', 2000).get_parameter_value().integer_value
		self.timeout = self.declare_parameter('server_connection.timeout', 10.0).get_parameter_value().double_value

		self.publisher = self.create_publisher(CarlaCollisionEvent, '/carla/hero/collision', 10)

		self.client = carla.Client(self.server_host, self.server_port)
		self.client.set_timeout(self.timeout)
		self.world = self.client.get_world()

		self.hero_vehicle = None
		self.collision_sensor = None

		self.setup_collision_sensor()
  
	def setup_collision_sensor(self):
		# Find the hero vehicle
		if self.hero_vehicle is None:
			self.hero_vehicle = carla_utils.get_hero_vehicle(self.world, self.timeout)
			if self.hero_vehicle is None:
				self.get_logger().error('Hero vehicle not found; collision sensor will not be created.')
				return

		# Set up the collision sensor
		blueprint_library = self.world.get_blueprint_library()
		collision_bp = blueprint_library.find('sensor.other.collision')

		# Spawn the sensor and attach it to the hero vehicle
		spawn_point = carla.Transform(carla.Location(0, 0, 0))
		self.collision_sensor = self.world.spawn_actor(collision_bp, spawn_point, attach_to=self.hero_vehicle)
		self.get_logger().info('Spawning sensor: {}'.format(self.collision_sensor.type_id))

		def on_collision(event):
			msg = CarlaCollisionEvent()
			msg.header.stamp = self.get_clock().now().to_msg()
			msg.other_actor_id = event.other_actor.id
			msg.normal_impulse.x = event.normal_impulse.x
			msg.normal_impulse.y = event.normal_impulse.y
			msg.normal_impulse.z = event.normal_impulse.z
			self.publisher.publish(msg)

		self.collision_sensor.listen(on_collision)

	def destroy_node(self):
		if self.collision_sensor is not None:
			if self.collision_sensor.is_listening:
				self.collision_sensor.stop()
			if self.collision_sensor.is_alive:
				self.collision_sensor.destroy()
			self.collision_sensor = None

		super().destroy_node()


def main():
	rclpy.init()

	node = CollisionSensor()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()


if __name__ == '__main__':
	main()
