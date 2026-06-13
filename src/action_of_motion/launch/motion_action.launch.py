from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    default_params_file = PathJoinSubstitution([
        FindPackageShare('action_of_motion'),
        'config',
        'param.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the motion action parameter file.',
        ),
        Node(
            package='action_of_motion',
            executable='motion_action_node',
            name='motion_action_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
