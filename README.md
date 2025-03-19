# CARLA Interface

ROS package to interface with CARLA topics and set up the CARLA simulation environment.

ROS Version: `ROS1 Noetic`

## Running the Interface
After cloning the package into your ROS workspace src directory and building your workspace, launch the interface with: 

`roslaunch carla_shell_bridge main.launch`

## Configuring CARLA
Below is a list of all the different parameters that can be change to configure the CARLA simulator to help test different scenarios or enhance simulation performance. The parameters can be changed in [`carla_interface/launch/main.launch`](launch/main.launch)

### Server Connection Parameters:

`host`: IP of the CARLA server

`port`: Carla server port

`passive`: Toggles passive mode for the simulation

`synchronous_mode`: Toggles the simulation from syncronous mode to asynchronous mode

`synchronous_mode_wait_for_vehicle_control_command`: Toggles the simulation to syncronous mode and waits for a vehicle command

`fixed_delta_seconds`: Indirectly controls the speed of the simulation

`timeout`: Server timeout limit (seconds)

### Server Environment Parameters:

`map`: Map that is loaded on startup (either a predefined CARLA town (e.g. 'Town01_Opt'), or a OpenDRIVE map file)

#### Map Layers
Layers of the map that can be added or removed to improve performance (Note: Only works with "Opt" maps)

`all`: Set to True to load all map layers regardless of values below

`decals`: Toggle's decals

`foliage`: Toggle's foliage

`parked_vehicles`: Toggle's parked_vehicles

`particles`: Toggle's particles

`walls`: Toggle's walls

### Traffic Generation Parameters:

`number_of_vehicles`: Number of other vehicles to spawn in the simulations

`spawn_radius`: Spawn radius from ego vehicle in meters

`seed`: Set random device seed and deterministic mode for Traffic Manager

### Ego Vehicle Parameters:

`role_name`: Role name of the ego vehicle

`spawn_point_ego_vehicle`: "x,y,z,roll,pitch,yaw" map coordinates to spawn the vehicle (Use "None" to spawn the ego vehicle in a random point on the road)

`spawn_sensors_only`: Set to true to only spawn the vehicle sensors
