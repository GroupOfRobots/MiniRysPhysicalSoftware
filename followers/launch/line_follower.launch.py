import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('followers'),
        'config',
        'params.yaml'
    )

    namespace_value = LaunchConfiguration('minirys_namespace')
    arg_namespace = DeclareLaunchArgument('minirys_namespace', default_value='minirys')
    node = Node(
        package = 'followers',
        name = 'line_follower',
        namespace=namespace_value,
        executable = 'linefollower.py',
        parameters = [config]
    )

    return LaunchDescription([arg_namespace, node])
