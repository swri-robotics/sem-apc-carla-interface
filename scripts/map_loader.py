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
import rospy
import time
import math

class CarlaMapConfig:
  def __init__(self):
    rospy.init_node('map_loader')

    # Parameters
    self.server_host = rospy.get_param('/map_loader/host', default='localhost')
    self.server_port = rospy.get_param('/map_loader/port', default=2000)
    self.timeout = rospy.get_param('/map_loader/timeout', default=10.0)
    self.load_all = rospy.get_param('/map_loader/all', default=False)
    self.load_buildings = rospy.get_param('/map_loader/buildings', default=True)
    self.load_decals = rospy.get_param('/map_loader/decals', default=False)
    self.load_foliage = rospy.get_param('/map_loader/foliage', default=True)
    self.load_ground = rospy.get_param('/map_loader/ground', default=True)
    self.load_parked_vehicles = rospy.get_param('/map_loader/parked_vehicles', default=False)
    self.load_particles = rospy.get_param('/map_loader/particles', default=False)
    self.load_props = rospy.get_param('/map_loader/props', default=True)
    self.load_street_lights = rospy.get_param('/map_loader/street_lights', default=True)
    self.load_walls = rospy.get_param('/map_loader/walls', default=True)

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
      rospy.loginfo("Loading default map layers.")
      return

    rospy.loginfo("Removing selected map layers.")
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
    rospy.loginfo("Setting spectator behind ego vehicle.")
    try:
      # Find the ego vehicle
      ego_vehicle = self.get_ego_vehicle(self.timeout)
      # Wait for the ego vehicle to drop before setting the spectator view position
      time.sleep(1.0)
      # Setup the spectator camera
      spectator = self.world.get_spectator()
      spectator.set_transform(self.get_spectator_transform(ego_vehicle.get_transform()))
    except:
      rospy.logwarn("Ego vehicle did not spawn correctly! Cannot move camera behind vehicle.")
      
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
          rospy.loginfo(f"Ego vehicle: {vehicle.get_transform()}")
          return vehicle
      # Check if we've timed out
      if time.time() - start > timeout:
        break
      # Wait a second to see if ego vehicle will spawn
      time.sleep(1.0)
        
    return None


def main():
  try:
    # Initialize ROS node
    carla_map_config_node = CarlaMapConfig()
    carla_map_config_node.map_unload()
    carla_map_config_node.set_spectator_view()
  except rospy.ROSInterruptException:
    pass

if __name__ == "__main__":  
  try: 
    main()
  except KeyboardInterrupt:
    pass