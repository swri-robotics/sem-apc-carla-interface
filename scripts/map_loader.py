#!/usr/bin/env python3

import carla
import rclpy
from rclpy.node import Node
import time
import math

"""
This node loads the CARLA map, unloads/loads map layers, and sets the server spectator camera behind the ego vehicle on spawn.
"""
class CarlaMapConfig(Node):
    def __init__(self):
        super().__init__('map_loader')

        # Parameters
        self.server_host = self.declare_parameter('server_connection.host', 'localhost').get_parameter_value().string_value
        self.server_port = self.declare_parameter('server_connection.port', 2000).get_parameter_value().integer_value
        self.timeout = self.declare_parameter('server_connection.timeout', 10.0).get_parameter_value().double_value
        self.map = self.declare_parameter('server_environment.map', 'Town05_Opt').get_parameter_value().string_value
        self.load_all = self.declare_parameter('server_environment.map_layers.all', False).get_parameter_value().bool_value
        self.load_buildings = self.declare_parameter('server_environment.map_layers.buildings', True).get_parameter_value().bool_value
        self.load_decals = self.declare_parameter('server_environment.map_layers.decals', False).get_parameter_value().bool_value
        self.load_foliage = self.declare_parameter('server_environment.map_layers.foliage', True).get_parameter_value().bool_value
        self.load_ground = self.declare_parameter('server_environment.map_layers.ground', True).get_parameter_value().bool_value
        self.load_parked_vehicles = self.declare_parameter('server_environment.map_layers.parked_vehicles', False).get_parameter_value().bool_value
        self.load_particles = self.declare_parameter('server_environment.map_layers.particles', False).get_parameter_value().bool_value
        self.load_props = self.declare_parameter('server_environment.map_layers.props', True).get_parameter_value().bool_value
        self.load_street_lights = self.declare_parameter('server_environment.map_layers.street_lights', True).get_parameter_value().bool_value
        self.load_walls = self.declare_parameter('server_environment.map_layers.walls', True).get_parameter_value().bool_value
        self.load_none = self.declare_parameter('server_environment.map_layers.none', False).get_parameter_value().bool_value

        # Setup CARLA
        self.client = carla.Client(self.server_host, self.server_port)
        self.client.set_timeout(self.timeout)
        self.client.load_world(self.map, map_layers=carla.MapLayer.NONE)
        self.world = self.client.get_world()
        self.settings = self.world.get_settings()
        self.world.wait_for_tick()
  
    def load_map_layers(self):
        """
        Load desired layers of the map
        """
        self.get_logger().info("Loading selected map layers.")

        if self.load_none:
            return
        if self.load_all:
            self.world.load_map_layer(carla.MapLayer.All)
            self.world.wait_for_tick()
            return
        if self.load_buildings:
            self.world.load_map_layer(carla.MapLayer.Buildings)
        if self.load_decals:
            self.world.load_map_layer(carla.MapLayer.Decals)
        if self.load_foliage:
            self.world.load_map_layer(carla.MapLayer.Foliage)
        if self.load_ground:
            self.world.load_map_layer(carla.MapLayer.Ground)
        if self.load_parked_vehicles:
            self.world.load_map_layer(carla.MapLayer.ParkedVehicles)
        if self.load_particles:
            self.world.load_map_layer(carla.MapLayer.Particles)
        if self.load_props:
            self.world.load_map_layer(carla.MapLayer.Props)
        if self.load_street_lights:
            self.world.load_map_layer(carla.MapLayer.StreetLights)
        if self.load_walls:
            self.world.load_map_layer(carla.MapLayer.Walls)
        self.world.wait_for_tick()

def main():
    # Initialize ROS node
    rclpy.init()

    carla_map_config_node = CarlaMapConfig()
    carla_map_config_node.load_map_layers()

    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass