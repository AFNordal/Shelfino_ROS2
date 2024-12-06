from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shelfino_motion',
            executable='motion_node',
            name='motion_controller',
            output='screen'
        )
    ])
