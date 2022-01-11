from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parameters',
            executable='parameters_node',
            name='parameters',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                }
            ]
        )
    ])
