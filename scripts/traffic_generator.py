#!/usr/bin/env python3

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

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
from carla import VehicleLightState as vls
import rclpy
from rclpy.node import Node
import time
import logging
from numpy import random

"""
This node spawns NPC vehicles in the CARLA environment.
"""
class TrafficGenerator(Node):
    def __init__(self):
        super().__init__('traffic_generator')
        
        # Parameters
        self.server_host = self.declare_parameter('server_connection.host', 'localhost').get_parameter_value().string_value
        self.server_port = self.declare_parameter('server_connection.port', 2000).get_parameter_value().integer_value
        self.timeout = self.declare_parameter('server_connection.timeout', 10.0).get_parameter_value().double_value
        self.number_of_vehicles = self.declare_parameter('server_environment.traffic_generation.number_of_vehicles', 5).get_parameter_value().integer_value
        self.spawn_radius = self.declare_parameter('server_environment.traffic_generation.spawn_radius', 100.0).get_parameter_value().double_value
        self.traffic_seed = self.declare_parameter('server_environment.traffic_generation.seed', 0).get_parameter_value().integer_value

        # Parameters not yet exposed to ROS
        self.traffic_manager_port = 8000
        self.respawn = False
        self.hybrid = None
        self.vehicle_filter = 'vehicle.*'
        self.vehicle_gen = 'All'
        self.safe_spawn = True
        self.car_lights_on = False
        self.asynch = None
        
        # Wait to make sure new map has been loaded by ros_bridge
        time.sleep(3.0)
        
        # Setup CARLA
        self.client = carla.Client(self.server_host, self.server_port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()
        self.settings = self.world.get_settings()
        self.world.wait_for_tick()

    def get_actor_blueprints(self, filter, generation):
        """
        Get actor blueprints for spawning
        """
        bps = self.world.get_blueprint_library().filter(filter)

        if generation.lower() == "all":
            return bps

        # If the filter returns only one bp, we assume that this one needed
        # and therefore, we ignore the generation
        if len(bps) == 1:
            return bps

        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            self.get_logger().warn("Actor Generation is not valid! No actor will be spawned.")
            return []
            
    def spawn_vehicles(self):
        """
        Generate vehicles in the CARLA simulator
        """
        vehicles_list = []
        synchronous_master = False

        try:
            traffic_manager = self.client.get_trafficmanager(self.traffic_manager_port)
            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            if self.respawn:
                traffic_manager.set_respawn_dormant_vehicles(True)
            if self.hybrid:
                traffic_manager.set_hybrid_physics_mode(True)
                traffic_manager.set_hybrid_physics_radius(70.0)
            if self.traffic_seed is not None:
                traffic_manager.set_random_device_seed(self.traffic_seed)

            blueprints = self.get_actor_blueprints(self.vehicle_filter, self.vehicle_gen)
            
            if self.safe_spawn:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
                blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
                blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('t2')]
                blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
                blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

            blueprints = sorted(blueprints, key=lambda bp: bp.id)

            spawn_points = self.world.get_map().get_spawn_points()
            filtered_spawn_points = []
      
            ego_vehicle = self.get_ego_vehicle(self.timeout)
            ego_vehicle_spawn = ego_vehicle.get_transform()
            
            # Filter spawn points so that we only get spawn points near the ego vehicle
            for point in spawn_points:
                # Add spawn point if it's close enough to the ego vehicle
                if point.location.distance(ego_vehicle_spawn.location) < self.spawn_radius:
                    filtered_spawn_points.append(point)
            
            number_of_spawn_points = len(filtered_spawn_points)

            if self.number_of_vehicles < number_of_spawn_points:
                random.shuffle(filtered_spawn_points)
            elif self.number_of_vehicles > number_of_spawn_points:
                msg = 'requested %d vehicles, but could only find %d spawn points'
                logging.warning(msg, self.number_of_vehicles, number_of_spawn_points)
                self.number_of_vehicles = number_of_spawn_points

            # @todo cannot import these directly.
            SpawnActor = carla.command.SpawnActor
            SetAutopilot = carla.command.SetAutopilot
            SetVehicleLightState = carla.command.SetVehicleLightState
            FutureActor = carla.command.FutureActor

            # --------------
            # Spawn vehicles
            # --------------
            batch = []
            for n, transform in enumerate(filtered_spawn_points):
                if n >= self.number_of_vehicles:
                    break
                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)
                else:
                    blueprint.set_attribute('role_name', 'autopilot')

                # prepare the light state of the cars to spawn
                light_state = vls.NONE
                if self.car_lights_on:
                    light_state = vls.Position | vls.LowBeam | vls.LowBeam

                # spawn the cars and set their autopilot and light state all together
                batch.append(SpawnActor(blueprint, transform)
                    .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
                    .then(SetVehicleLightState(FutureActor, light_state)))

            for response in self.client.apply_batch_sync(batch, synchronous_master):
                if response.error:
                    logging.error(response.error)
                else:
                    vehicles_list.append(response.actor_id)

            self.get_logger().info('Spawned %d vehicles.' % (len(vehicles_list)))

            # Example of how to use Traffic Manager parameters
            traffic_manager.global_percentage_speed_difference(30.0)

            while True:
                if not self.asynch and synchronous_master:
                    self.world.tick()
                else:
                    self.world.wait_for_tick()

        finally:
            
            if not self.asynch:
                self.settings.synchronous_mode = False
                self.world.apply_settings(self.settings)

            self.get_logger().info('\ndestroying %d vehicles' % len(vehicles_list))
            self.client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

            time.sleep(0.5)
    
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
                    return vehicle
            # Check if we've timed out
            if time.time() - start > timeout:
                break
            # Wait a second to see if ego vehicle will spawn
            time.sleep(1.0)
        return None

def main():
    # Initialize ROS Node
    rclpy.init()
    
    traffic_gen_node = TrafficGenerator()
    traffic_gen_node.spawn_vehicles()
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass