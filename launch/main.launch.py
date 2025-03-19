from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import yaml

package_name = 'carla_shell_bridge'

def generate_launch_description():
   
    # Get simulation config parameters from yaml file 
    carla_config_file_path = os.path.expanduser('~/shell_ws/src/sem-apc-carla-interface/config/carla_config.yaml')
    with open(carla_config_file_path, 'r') as f:
        config = yaml.safe_load(f)
        config = config['/**']['ros__parameters']

    # Arguments
    
    ##### CARLA server connection args #####
    
    # server_url = LaunchConfiguration('server_url')
    # server_url_arg = DeclareLaunchArgument('server_url', default_value='$(env CARLA_SERVER)')

    config_file = LaunchConfiguration('config_file')
    config_file_arg = DeclareLaunchArgument('config_file', default_value=[carla_config_file_path],
                                            description='Name of config_file file.')

    host = LaunchConfiguration('host')
    host_arg = DeclareLaunchArgument('host', default_value=config['server_connection']['host'], 
                                     description='IP to connect to CARLA server (set to \'localhost\' if running server locally)')

    port = LaunchConfiguration('port')
    port_arg = DeclareLaunchArgument('port', default_value=str(config['server_connection']['port']), 
                                     description='Server port (usually 2000)')

    timeout = LaunchConfiguration('timeout')
    timeout_arg = DeclareLaunchArgument('timeout', default_value=str(config['server_connection']['timeout']), 
                                        description='Server timeout limit (seconds)')
    
    ##### Ego vehicle args #####
    
    role_name = LaunchConfiguration('role_name')
    role_name_arg = DeclareLaunchArgument('role_name', default_value=config['ego_vehicle']['role_name'])

    vehicle_filter = LaunchConfiguration('vehicle_filter')
    vehicle_filter_arg = DeclareLaunchArgument('vehicle_filter', default_value='vehicle.*')

    objects_definition_file = LaunchConfiguration('objects_definition_file')
    objects_definition_file_arg = DeclareLaunchArgument('objects_definition_file', default_value=[os.path.join(get_package_share_directory('carla_shell_bridge'), 'launch'),
                                       '/objects.json'])    

    # Use comma separated format "x,y,z,roll,pitch,yaw", and parameter name spawn_point_<vehicle_name>. You can add
    # as many spawn_point as vehicles defined in objects_definition_file
    spawn_point_ego_vehicle = LaunchConfiguration('spawn_point_ego_vehicle')
    spawn_point_ego_vehicle_arg = DeclareLaunchArgument('spawn_point_ego_vehicle', default_value=config['ego_vehicle']['spawn_point_ego_vehicle']) #-88.710991, -119.565231, 0.275307, 0.275307, 89.843742, 0.0

    spawn_sensors_only = LaunchConfiguration('spawn_sensors_only')
    spawn_sensors_only_arg = DeclareLaunchArgument('spawn_sensors_only', default_value=str(config['ego_vehicle']['spawn_sensors_only']))
    
    ##### Map args #####
    
    town = LaunchConfiguration('town')
    town_arg = DeclareLaunchArgument('town', default_value=config['server_environment']['map'], description='Map to load on startup (either a predefined CARLA town (e.g. \'Town03\'), or a OpenDRIVE map file)')
    
    ##### Miscellaneous args #####
    
    # Enable/disable passive mode
    passive = LaunchConfiguration('passive')
    passive_arg = DeclareLaunchArgument('passive', default_value=str(config['server_connection']['passive']))

    # Synchronous mode
    synchronous_mode_wait_for_vehicle_control_command = LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')
    synchronous_mode_wait_for_vehicle_control_command_arg = DeclareLaunchArgument('synchronous_mode_wait_for_vehicle_control_command', default_value=str(config['server_connection']['synchronous_mode_wait_for_vehicle_control_command']))

    synchronous_mode = LaunchConfiguration('synchronous_mode')
    synchronous_mode_arg = DeclareLaunchArgument('synchronous_mode', default_value=str(config['server_connection']['synchronous_mode']))

    fixed_delta_seconds = LaunchConfiguration('fixed_delta_seconds')
    fixed_delta_seconds_arg = DeclareLaunchArgument('fixed_delta_seconds', default_value=str(config['server_connection']['fixed_delta_seconds']))


    # Nodes
    carla_ros_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('carla_ros_bridge')),
                                       '/carla_ros_bridge.launch.py']),
            launch_arguments={
                'host': host,
                'port': port,
                'town': town,
                'timeout': timeout,
                'passive': passive,
                'synchronous_mode': synchronous_mode,
                'synchronous_mode_wait_for_vehicle_control_command': synchronous_mode_wait_for_vehicle_control_command,
                'fixed_delta_seconds': fixed_delta_seconds,
            }.items(),
    )
    
    ego_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('carla_spawn_objects')),
                                       '/carla_example_ego_vehicle.launch.py']),
            launch_arguments={
                'objects_definition_file': objects_definition_file,
                'role_name': role_name,
                'spawn_point_ego_vehicle': spawn_point_ego_vehicle,
                'spawn_sensors_only': spawn_sensors_only,
            }.items(),
    )
    
    carla_shell_interface = ComposableNodeContainer(
        name='carla_shell_interface_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='carla_shell_bridge',
                plugin='carla_shell_bridge::CarlaSimulationVehicleInterface',
                name='carla_interface',
                parameters=[config_file],
                remappings=[('/car_cmd', '/carla/ego_vehicle/vehicle_control_cmd')]
            ),
        ],
        output='both'
    )
    
    map_loader_node = Node(
        package='carla_shell_bridge',
        namespace='',
        executable='map_loader.py',
        name='map_loader',
        parameters=[config_file],
        output='screen',
    )
    
    traffic_gen_node = Node(
        package='carla_shell_bridge',
        namespace='',
        executable='traffic_generator.py',
        name='traffic_generator',
        parameters=[config_file],
        output='screen',
    )
    
    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2',
       arguments=['-d' + os.path.expanduser('~/shell_ws/src/sem-apc-carla-interface/config/default_config.rviz')],
    )
    
    
    return LaunchDescription([
        
        # Arguments
        config_file_arg,
        host_arg,
        port_arg,
        timeout_arg,
        role_name_arg,
        vehicle_filter_arg, 
        objects_definition_file_arg,
        spawn_point_ego_vehicle_arg,
        spawn_sensors_only_arg,
        town_arg,
        passive_arg,
        synchronous_mode_wait_for_vehicle_control_command_arg,
        synchronous_mode_arg,
        fixed_delta_seconds_arg,
        
        # Nodes
        carla_ros_bridge,
        ego_vehicle,
        carla_shell_interface,
        map_loader_node,
        traffic_gen_node,
        rviz_node
    ])