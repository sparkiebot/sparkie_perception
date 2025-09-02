from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch file argument declarations
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable LIDAR sensor'
    )
    
    use_odom_arg = DeclareLaunchArgument(
        'use_odom',
        default_value='true',
        description='Enable odometry calculation'
    )
    
    use_imu_filter_arg = DeclareLaunchArgument(
        'use_imu_filter',
        default_value='true',
        description='Enable IMU Madgwick filter'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar_link',
        description='LIDAR frame ID'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID'
    )
    
    # Include LIDAR launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sparkie_lidar'),
                'launch',
                'lidar.launch.py'
            ])
        ]),
        launch_arguments={
            'frame_id': LaunchConfiguration('lidar_frame'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # Include odometry launch file
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sparkie_odom'),
                'launch',
                'odom.launch.py'
            ])
        ]),
        launch_arguments={
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_odom'))
    )
        
    return LaunchDescription([
        # Arguments
        use_lidar_arg,
        use_odom_arg,
        use_imu_filter_arg,
        lidar_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        
        # Launch files
        lidar_launch,
        odom_launch,
    ])