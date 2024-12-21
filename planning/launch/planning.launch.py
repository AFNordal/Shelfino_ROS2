from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    strategy_value = LaunchConfiguration('strategy')
 
    strategy_arg = DeclareLaunchArgument(
        'strategy',
        default_value='combinatorial'
    )

    return LaunchDescription([
        strategy_arg,
        Node(
            package="planning",
            executable="roadmap_gen",
            name="roadmap_generator",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"strategy": strategy_value}
            ]
        )
    ])