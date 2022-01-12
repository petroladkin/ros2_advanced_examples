from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='actions',
            executable='action_server_node',
            name='actions',
            output='screen',
            emulate_tty=True,
            parameters=[{}]
        )
    ])
