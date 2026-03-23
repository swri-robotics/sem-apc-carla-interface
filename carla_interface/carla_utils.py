import carla
import time
import math


def get_hero_vehicle(world, timeout=10.0):
    """
    Get the hero vehicle once it has spawned
    """
    start = time.time()
    # Wait for Hero vehicle to spawn
    while True:
        # Filter out everything except vehicle actors
        vehicle_list = world.get_actors().filter('vehicle.*')
        # Find the hero vehicle
        for vehicle in vehicle_list:
        # Hero vehicle found
            if vehicle.attributes['role_name'] == 'hero':
                return vehicle
        # Check if we've timed out
        if time.time() - start > timeout:
            break
        # Wait a second to see if hero vehicle will spawn
        time.sleep(1.0)
        
    return None

def get_spectator_transform(vehicle_transform, d=5):
    """
    Get the tranform between the vehicle and the spectator
    """
    a = math.radians(vehicle_transform.rotation.yaw)
    location = carla.Location(-d * math.cos(a), -d * math.sin(a), 2.5) + vehicle_transform.location
    return carla.Transform(location, carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw))


def set_spectator_view(vehicle, world):
    """
    Set the spectator camera to start behind the hero vehicle
    """
    try:
        # Wait for the hero vehicle to drop before setting the spectator view position
        time.sleep(1.0)
        # Setup the spectator camera
        spectator = world.get_spectator()
        spectator.set_transform(get_spectator_transform(vehicle.get_transform()))
        return True
    except:
        return False