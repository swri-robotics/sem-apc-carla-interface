#!/usr/bin/env python3
"""
ROS node generates hero vehicle and other npc vehicles in the CARLA simulator 
and sets up sensors for the hero vehicle.
"""

import json
import math
import os

import carla
from carla import VehicleLightState as vls
from carla.command import SpawnActor, SetAutopilot, FutureActor, DestroyActor

import rclpy
from rclpy.node import Node

import logging
from numpy import random
import time
import math

from carla_interface import carla_utils

class VehicleGenerator(Node):
    def __init__(self):
        super().__init__('vehicle_generator')
        
        # Parameters
        self.server_host = self.declare_parameter('server_connection.host', 'localhost').get_parameter_value().string_value
        self.server_port = self.declare_parameter('server_connection.port', 2000).get_parameter_value().integer_value
        self.timeout = self.declare_parameter('server_connection.timeout', 10.0).get_parameter_value().double_value
        self.number_of_vehicles = self.declare_parameter('server_environment.traffic_generation.number_of_vehicles', 5).get_parameter_value().integer_value + 1 # +1 for the hero vehicle
        self.traffic_seed = self.declare_parameter('server_environment.traffic_generation.seed', -1).get_parameter_value().integer_value
        self.spawn_point_hero_vehicle = self.declare_parameter('hero_vehicle.spawn_point_hero_vehicle', '0.0, 2.5, 0.2, 0, 0, 0').get_parameter_value().string_value

        # Parameters not yet exposed to ROS
        self.tm_port = 8000
        self.respawn = False
        self.hybrid = False
        self.asynch = False
        self.no_rendering = False
        self.vehicle_filter = 'vehicle.*'
        self.vehicle_gen = 'all'
        self.enable_blacklist = True
        # Vehicle models that are not allowed to be spawned in the traffic generation because 
        # they are not suitable for the competition
        self.blacklisted_veh_models = ['microlino', 
                                'carlacola', 
                                't2', 
                                'cybertruck', 
                                'sprinter', 
                                'firetruck', 
                                'ambulance',
                                'fusorosa']
        self.car_lights_on = False
        self.hero = True
        self.seed = 0
   

    def setup_sensors(self, world, vehicle, sensors_config):
        bp_library = world.get_blueprint_library()

        sensors = []
        for sensor in sensors_config:
            self.get_logger().info("Spawning sensor: {}".format(sensor.get("type")))

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


    def get_actor_blueprints(self, world, filter, generation):
        bps = world.get_blueprint_library().filter(filter)

        if generation.lower() == "all":
            return bps

        # If the filter returns only one bp, we assume that this one needed
        # and therefore, we ignore the generation
        if len(bps) == 1:
            return bps

        try:
            int_generation = int(generation)
            # Check if generation is in available generations
            if int_generation in [1, 2, 3]:
                bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
                return bps
            else:
                self.get_logger().warn("Actor Generation is not valid. No actor will be spawned.")
                return []
        except:
            self.get_logger().warn("Actor Generation is not valid. No actor will be spawned.")
            return []


    def spawn_vehicles(self):
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

        with open(os.path.expanduser('~/shell_ws/src/sem-apc-carla-interface/config/objects.json')) as f:
            config = json.load(f)

        vehicles_list = []
        client = carla.Client(self.server_host, self.server_port)
        client.set_timeout(self.timeout)
        synchronous_master = False
        random.seed(self.traffic_seed if self.traffic_seed != -1 else int(time.time()))

        try:
            world = client.get_world()

            traffic_manager = client.get_trafficmanager(self.tm_port)
            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            if self.respawn:
                traffic_manager.set_respawn_dormant_vehicles(True)
            if self.hybrid:
                traffic_manager.set_hybrid_physics_mode(True)
                traffic_manager.set_hybrid_physics_radius(70.0)
            if self.traffic_seed != -1:
                traffic_manager.set_random_device_seed(self.traffic_seed)

            settings = world.get_settings()
            if not self.asynch:
                traffic_manager.set_synchronous_mode(True)
                if not settings.synchronous_mode:
                    synchronous_master = True
                    settings.synchronous_mode = True
                    settings.fixed_delta_seconds = 0.05
                else:
                    synchronous_master = False
            else:
                self.get_logger().warn("You are currently in asynchronous mode. If this is a traffic simulation, \
                you could experience some issues. If it's not working correctly, switch to synchronous \
                mode by using traffic_manager.set_synchronous_mode(True)")

            if self.no_rendering:
                settings.no_rendering_mode = True
            world.apply_settings(settings)

            blueprints = self.get_actor_blueprints(world, self.vehicle_filter, self.vehicle_gen)
            if not blueprints:
                raise ValueError("Couldn't find any vehicles with the specified filters")
            if self.enable_blacklist:
                blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']
                for model in self.blacklisted_veh_models:
                    blueprints = [x for x in blueprints if not x.id.endswith(model)]

            blueprints = sorted(blueprints, key=lambda bp: bp.id)

            spawn_points = world.get_map().get_spawn_points()
            number_of_spawn_points = len(spawn_points)

            if self.number_of_vehicles < number_of_spawn_points:
                random.shuffle(spawn_points)
            elif self.number_of_vehicles > number_of_spawn_points:
                msg = 'requested %d vehicles, but could only find %d spawn points'
                logging.warning(msg, self.number_of_vehicles, number_of_spawn_points)
                self.number_of_vehicles = number_of_spawn_points

            # --------------
            # Spawn vehicles
            # --------------
            batch = []
            for n, transform in enumerate(spawn_points):
                if n >= self.number_of_vehicles:
                    break
                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)
                # Set up hero vehicle attributes
                if self.hero:
                    blueprint = world.get_blueprint_library().filter(config.get("type"))[0]
                    blueprint.set_attribute('role_name', config.get("id"))
                    blueprint.set_attribute("ros_name", config.get("id")) 
                    blueprint.set_attribute('color', '190,10,20')
                else:
                    blueprint.set_attribute('role_name', 'autopilot')

                # Spawn the hero vehicle
                if self.hero:
                    if self.spawn_point_hero_vehicle.lower() != "none":
                        self.get_logger().info("Spawning hero vehicle at custom spawn point: " + self.spawn_point_hero_vehicle)
                        spawn = [float(x) for x in self.spawn_point_hero_vehicle.split(',')]
                        hero_transform = carla.Transform(location=carla.Location(x=spawn[0], y=spawn[1], z=spawn[2]),
                                rotation=carla.Rotation(roll=spawn[3], pitch=spawn[4], yaw=spawn[5]))
                    else:
                        self.get_logger().info("Spawning hero vehicle at default spawn point of the map")
                        hero_transform = transform
                        
                    batch.append(SpawnActor(blueprint, hero_transform)
                        .then(SetAutopilot(FutureActor, False, traffic_manager.get_port())))
                    self.hero = False
                # Spawn the rest of the vehicles
                else:
                    batch.append(SpawnActor(blueprint, transform)
                        .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

            for response in client.apply_batch_sync(batch, synchronous_master):
                if response.error:
                    logging.error(response.error)
                else:
                    vehicles_list.append(response.actor_id)

            # Set automatic vehicle lights update if specified
            if self.car_lights_on:
                all_vehicle_actors = world.get_actors(vehicles_list)
                for actor in all_vehicle_actors:
                    traffic_manager.update_vehicle_lights(actor, True)

            self.get_logger().info('Spawned Hero vehicle and %d other vehicles' % (len(vehicles_list) - 1))

            sensors = self.setup_sensors(world, carla_utils.get_hero_vehicle(world), config.get("sensors", []))

            carla_utils.set_spectator_view(carla_utils.get_hero_vehicle(world), world)

            while True:
                if not self.asynch and synchronous_master:
                    world.tick()
                else:
                    world.wait_for_tick()

        finally:

            if not self.asynch and synchronous_master:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.no_rendering_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)

            self.get_logger().info('Destroying %d vehicles' % len(vehicles_list))
            client.apply_batch([DestroyActor(x) for x in sensors])
            client.apply_batch([DestroyActor(x) for x in vehicles_list])

            time.sleep(0.5)


if __name__ == '__main__':

    try:
        # Initialize ROS Node
        rclpy.init()
        
        vehicle_gen_node = VehicleGenerator()
        vehicle_gen_node.spawn_vehicles()
    
        rclpy.shutdown()
    
    except KeyboardInterrupt:
        pass