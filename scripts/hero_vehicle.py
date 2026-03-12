#!/usr/bin/env python3

import json
import logging
import os
import carla
import rclpy
from rclpy.node import Node
import time
import math

"""
This node spawns the hero vehicle that will be controlled by the teams in the CARLA environment.
"""
class HeroVehicle(Node):
    def __init__(self):
        super().__init__('hero_vehicle')
        
        # Parameters
        self.server_host = self.declare_parameter('server_connection.host', 'localhost').get_parameter_value().string_value
        self.server_port = self.declare_parameter('server_connection.port', 2000).get_parameter_value().integer_value
        self.timeout = self.declare_parameter('server_connection.timeout', 10.0).get_parameter_value().double_value
        self.spawn_point_ego_vehicle = self.declare_parameter('ego_vehicle.spawn_point_ego_vehicle', '0.0, -2.5, 0.2, 0, 0, 0').get_parameter_value().string_value
        
        time.sleep(3.0) # Wait to make sure new map has been loaded
        
    def setup_vehicle(self, world, config):
        logging.debug("Spawning vehicle: {}".format(config.get("type")))

        bp_library = world.get_blueprint_library()

        bp = bp_library.filter(config.get("type"))[0]
        bp.set_attribute("role_name", config.get("id"))
        bp.set_attribute("ros_name", config.get("id")) 

        # Convert string ROS param to list of floats
        spawn = [float(x) for x in self.spawn_point_ego_vehicle.split(',')]

        return  world.spawn_actor(
            bp,
            carla.Transform(
                location=carla.Location(x=spawn[0], y=spawn[1], z=spawn[2]),
                rotation=carla.Rotation(roll=spawn[3], pitch=spawn[4], yaw=spawn[5])
            ),
            attach_to=None)

    
    def setup_sensors(self, world, vehicle, sensors_config):
        bp_library = world.get_blueprint_library()

        sensors = []
        for sensor in sensors_config:
            logging.debug("Spawning sensor: {}".format(sensor))

            bp = bp_library.filter(sensor.get("type"))[0]
            bp.set_attribute("ros_name", sensor.get("id")) 
            bp.set_attribute("role_name", sensor.get("id")) 
            for key, value in sensor.get("attributes", {}).items():
                bp.set_attribute(str(key), str(value))

            wp = carla.Transform(
                location=carla.Location(x=sensor["spawn_point"]["x"], y=-sensor["spawn_point"]["y"], z=sensor["spawn_point"]["z"]),
                rotation=carla.Rotation(roll=sensor["spawn_point"]["roll"], pitch=-sensor["spawn_point"]["pitch"], yaw=-sensor["spawn_point"]["yaw"])
            )

            sensors.append(
                world.spawn_actor(
                    bp,
                    wp,
                    attach_to=vehicle
                )
            )

            sensors[-1].enable_for_ros()

        return sensors

    def get_spectator_transform(self, vehicle_transform, d=5):
        """
        Get the tranform between the vehicle and the spectator
        """
        a = math.radians(vehicle_transform.rotation.yaw)
        location = carla.Location(-d * math.cos(a), -d * math.sin(a), 2.5) + vehicle_transform.location
        return carla.Transform(location, carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw))

    def set_spectator_view(self, vehicle, world):
        """
        Set the spectator camera to start behind the ego vehicle
        """
        self.get_logger().info("Setting spectator behind ego vehicle.")
        try:
            # Wait for the ego vehicle to drop before setting the spectator view position
            time.sleep(1.0)
            # Setup the spectator camera
            spectator = world.get_spectator()
            spectator.set_transform(self.get_spectator_transform(vehicle.get_transform()))
        except:
            self.get_logger().warn("Ego vehicle did not spawn correctly! Cannot move camera behind vehicle.")

    def run(self):

        world = None
        vehicle = None
        sensors = []
        original_settings = None

        try:
            client = carla.Client(self.server_host, self.server_port)
            client.set_timeout(self.timeout)

            world = client.get_world()

            original_settings = world.get_settings()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

            with open(os.path.expanduser('~/shell_ws/src/sem-apc-carla-interface/config/objects.json'), "r") as f:
                config = json.load(f)

            vehicle = self.setup_vehicle(world, config)
            sensors = self.setup_sensors(world, vehicle, config.get("sensors", []))

            _ = world.tick()

            self.set_spectator_view(vehicle, world)

            _ = world.tick()

            logging.info("Running...")

            while True:
                _ = world.tick()

        except KeyboardInterrupt:
            print('\nCancelled by user. Bye!')

        finally:
            if original_settings:
                world.apply_settings(original_settings)

            for sensor in sensors:
                sensor.destroy()

            if vehicle:
                vehicle.destroy()

def main():
    # Initialize ROS Node
    rclpy.init()
    
    hero_vehicle_node = HeroVehicle()
    hero_vehicle_node.run()
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass