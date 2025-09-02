import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Positioning function pack
    pkg_share = FindPackageShare(package='sparkie_slam').find('sparkie_slam')
    
    # Map resolution
    resolution = LaunchConfiguration('resolution', default='0.05')
    # Map publish period  
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Configuration file folder path
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # Configuration file
    mapping_config = LaunchConfiguration('mapping_config', default='mapping.lua')
    localization_config = LaunchConfiguration('localization_config', default='localization.lua')

    # Choose between mapping or localization
    use_mapping = LaunchConfiguration('use_mapping', default='true')

    map_file = LaunchConfiguration('map_file', default=os.path.join(pkg_share, 'maps', 'my_map.pbstream'))


    #Nodes
    # Cartographer node for mapping
    cartographer_mapping_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        condition=IfCondition(EqualsSubstitution(use_mapping, 'true')),
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', mapping_config],
        remappings=[
            ('/scan', '/sparkie/scan'),
            ('/odom', '/sparkie/odom/ekf'),
        ],
    )

    # Cartographer node for localization
    cartographer_localization_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        condition=IfCondition(EqualsSubstitution(use_mapping, 'false')),
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', localization_config,
                   '-load_state_filename', map_file],
        remappings=[
            ('/scan', '/sparkie/scan'),
            ('/odom', '/sparkie/odom/ekf'),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
        
    )

    #Launch file
    ld = LaunchDescription()
    ld.add_action(cartographer_mapping_node)
    ld.add_action(cartographer_localization_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld
