# CARLA Interface

ROS package to interface with CARLA topics and set up the CARLA simulation environment.

ROS Version: `ROS2 Humble`

## Running the Interface
After cloning the package into your ROS workspace src directory and building your workspace, launch the interface with: 

`ros2 launch carla_interface main.launch.py`

## Configuring CARLA
Below is a list of all the different parameters that can be changed to configure the CARLA simulator to help test different scenarios or enhance simulation performance. The parameters can be changed in [`config/carla_config.yaml`](config/carla_config.yaml)

### Server Connection Parameters:

`host`: IP of the CARLA server

`port`: Carla server port

`passive`: Toggles passive mode for the simulation

`synchronous_mode`: Toggles the simulation from synchronous mode to asynchronous mode

`synchronous_mode_wait_for_vehicle_control_command`: Toggles the simulation to synchronous mode and waits for a vehicle command

`fixed_delta_seconds`: Indirectly controls the speed of the simulation

`timeout`: Server timeout limit (seconds)

### Server Environment Parameters:

`map`: Map that is loaded on startup (either a predefined CARLA town (e.g. 'Town01_Opt'), or a OpenDRIVE map file)

#### Map Layers
Layers of the map that can be added or removed to improve performance 

> [!NOTE]
> This only works with maps ending in "Opt"

`all`: Set to True to load all map layers regardless of values below

`buildings`: Toggles buildings

`decals`: Toggles decals

`foliage`: Toggles foliage

`ground`: Toggles ground (excluding roads)

`parked_vehicles`: Toggles parked vehicles

`particles`: Toggles particles

`props`: Toggles props

`street_lamps`: Toggles street lamps

`walls`: Toggles walls

### Traffic Generation Parameters:

`number_of_vehicles`: Number of other vehicles to spawn in the simulations

`spawn_radius`: Spawn radius from hero vehicle in meters

`seed`: Set random device seed and deterministic mode for Traffic Manager

### Hero Vehicle Parameters:

`spawn_point_hero_vehicle`: "x,y,z,roll,pitch,yaw" map coordinates to spawn the vehicle (Use "None" to spawn the hero vehicle in a random point on the road)
