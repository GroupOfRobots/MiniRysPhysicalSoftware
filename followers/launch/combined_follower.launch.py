import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('followers'),
        'config',
        'params.yaml'
    )

    wall_f = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('followers'),
                'launch/wall_follower.launch.py'
            )
        )
    )

    line_f = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('followers'),
                'launch/line_follower.launch.py'
            )
        )
    )

    namespace_value = LaunchConfiguration('minirys_namespace')
    arg_namespace = DeclareLaunchArgument('minirys_namespace', default_value='minirys')
    node=Node(
        package = 'followers',
        name = 'combined_follower',
        executable = 'combinedFollower',
        parameters = [config, {'minirys_namespace': namespace_value}]
    )

    return LaunchDescription([arg_namespace, line_f, node, wall_f])
