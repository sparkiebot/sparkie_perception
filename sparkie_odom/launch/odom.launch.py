import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sparkie_odom').find('sparkie_odom')
    
    imu_filter_madgwick_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(pkg_share, 'params', 'imu.yaml')],
        remappings=[
            ('/imu/data_raw', '/sparkie/board/imu'),
            ('/imu/data', '/sparkie/imu'),
            #('/imu/mag', '/sparkie/board/mag'),
        ],
    )

    imu_filter_complementary_node = launch_ros.actions.Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(pkg_share, 'params', 'imu.yaml')],
        remappings=[
            ('/imu/data_raw', '/sparkie/board/imu'),
            ('/imu/data', '/sparkie/imu'),
            #('/imu/mag', '/sparkie/board/mag'),
        ],
    )

    ekf_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'params', 'ekf.yaml')],
        remappings=[
            ('/odometry/filtered', '/sparkie/odom/ekf'),
            ('/odom', '/sparkie/board/wheels/odom'),
            ('/imu', '/sparkie/imu'),
        ]
    )


    
    return launch.LaunchDescription([
        imu_filter_madgwick_node,
        #imu_filter_complementary_node,
        ekf_localization_node
    ])
