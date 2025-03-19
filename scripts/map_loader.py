#!/usr/bin/env python3

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

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

        # Wait to make sure new map has been loaded by ros_bridge
        time.sleep(3.0)

        # Setup CARLA
        self.client = carla.Client(self.server_host, self.server_port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()
        self.settings = self.world.get_settings()
        self.world.wait_for_tick()
  
    def map_unload(self):
        """
        Unload undesired layers of the map
        """
        if self.load_all:
            # Don't unload any map layers
            self.get_logger().info("Loading default map layers.")
            return

        self.get_logger().info("Removing selected map layers.")
        if not self.load_buildings:
            self.world.unload_map_layer(carla.MapLayer.Buildings)
        if not self.load_decals:
            self.world.unload_map_layer(carla.MapLayer.Decals)
        if not self.load_foliage:
            self.world.unload_map_layer(carla.MapLayer.Foliage)
        if not self.load_ground:
            self.world.unload_map_layer(carla.MapLayer.Ground)
        if not self.load_parked_vehicles:
            self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        if not self.load_particles:
            self.world.unload_map_layer(carla.MapLayer.Particles)
        if not self.load_props:
            self.world.unload_map_layer(carla.MapLayer.Props)
        if not self.load_street_lights:
            self.world.unload_map_layer(carla.MapLayer.StreetLights)
        if not self.load_walls:
            self.world.unload_map_layer(carla.MapLayer.Walls)

    def get_spectator_transform(self, vehicle_transform, d=5):
        """
        Get the tranform between the vehicle and the spectator
        """
        a = math.radians(vehicle_transform.rotation.yaw)
        location = carla.Location(-d * math.cos(a), -d * math.sin(a), 2.5) + vehicle_transform.location
        return carla.Transform(location, carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw))

    def set_spectator_view(self):
        """
        Set the spectator camera to start behind the ego vehicle
        """
        self.get_logger().info("Setting spectator behind ego vehicle.")
        try:
            # Find the ego vehicle
            ego_vehicle = self.get_ego_vehicle(self.timeout)
            # Wait for the ego vehicle to drop before setting the spectator view position
            time.sleep(1.0)
            # Setup the spectator camera
            spectator = self.world.get_spectator()
            spectator.set_transform(self.get_spectator_transform(ego_vehicle.get_transform()))
        except:
            self.get_logger().warn("Ego vehicle did not spawn correctly! Cannot move camera behind vehicle.")

    def get_ego_vehicle(self, timeout):
        """
        Get the ego vehicle once it has spawned
        """
        start = time.time()
        # Wait for Ego vehcile to spawn
        while True:
            # Get all actors from the world
            actor_list = self.world.get_actors()
            # Filter out everything except vehicle actors
            vehicle_list = actor_list.filter('vehicle.*')
            # Find the ego vehicle
            for vehicle in vehicle_list:
            # Ego vehicle found
                if vehicle.attributes['role_name'] == 'ego_vehicle':
                    self.get_logger().info(f"Ego vehicle: {vehicle.get_transform()}")
                    return vehicle
            # Check if we've timed out
            if time.time() - start > timeout:
                break
            # Wait a second to see if ego vehicle will spawn
            time.sleep(1.0)
            
        return None
  

def main():
    # Initialize ROS node
    rclpy.init()

    carla_map_config_node = CarlaMapConfig()
    carla_map_config_node.map_unload()
    carla_map_config_node.set_spectator_view()

    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass