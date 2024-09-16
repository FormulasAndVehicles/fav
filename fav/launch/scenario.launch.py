from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    arg = DeclareLaunchArgument('use_sim_time')
    launch_description.add_action(arg)

    scenario_arg = DeclareLaunchArgument(
        name='scenario',
        default_value=str(1),
        description='The number of the scenario',
    )
    launch_description.add_action(scenario_arg)

    rviz_file = str(
        get_package_share_path('final_project') / 'config/rviz.rviz'
    )

    group = GroupAction(
        [
            PushROSNamespace(LaunchConfiguration('vehicle_name')),
            Node(
                executable='scenario_node.py',
                package='fav',
                parameters=[
                    {
                        'scenario': LaunchConfiguration('scenario'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    },
                ],
            ),
            Node(
                executable='robot_marker_publisher.py',
                package='fav',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    },
                ],
            ),
        ]
    )
    launch_description.add_action(group)
    action = Node(
        executable='rviz2',
        package='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
    )
    launch_description.add_action(action)
    return launch_description
