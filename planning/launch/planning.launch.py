from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    strategy_value = LaunchConfiguration("strategy")
    skip_shelfino_value = LaunchConfiguration("skip_shelfino")

    strategy_arg = DeclareLaunchArgument("strategy", default_value="combinatorial")
    skip_shelfino_arg = DeclareLaunchArgument("skip_shelfino", default_value="false")

    mapgen_node = Node(
        package="planning",
        executable="roadmap_gen",
        name="roadmap_generator",
        output="screen",
        emulate_tty=True,
        parameters=[{"strategy": strategy_value, "skip_shelfino": skip_shelfino_value}],
    )

    planner_node = Node(
        package="planning",
        executable="taskplanning",
        name="task_planner",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription([strategy_arg, skip_shelfino_arg, mapgen_node, planner_node])
