import os

from ament_index_python.packages import get_package_share_directory
from launch import actions, events, event_handlers, substitutions, LaunchDescription
import launch_ros

def get_shutdown_on_exit(action):
	return actions.RegisterEventHandler(event_handler=event_handlers.OnProcessExit(
		target_action=action,
		on_exit=[actions.EmitEvent(event=events.Shutdown())]
	))

def generate_launch_description():
	namespace_value = os.environ.get('NAMESPACE')

	params_path = os.path.join(
		get_package_share_directory('minirys_ros2'),
		'config',
		namespace_value + '_params.yaml'
	)

	arg_namespace = actions.DeclareLaunchArgument('namespace', default_value='minirys')
	debug_arg = actions.DeclareLaunchArgument('debug', default_value='false')
	node_ve = launch_ros.actions.Node(
		package="minirys_ros2",
		executable="minirys_ve",
		namespace=namespace_value,
		parameters=[params_path]
	)

	return LaunchDescription([
		arg_namespace,
		node_ve,
		get_shutdown_on_exit(node_ve),
	])
