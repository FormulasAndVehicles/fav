from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_launch_args(launch_description: LaunchDescription) -> None:
    declare_vehicle_name_and_sim_time(
        launch_description=launch_description, use_sim_time_default='true'
    )


def include_simulation_launch(launch_description: LaunchDescription) -> None:
    package_path = get_package_share_path('fav')
    path = str(package_path / 'launch/simulation_with_tags.launch.py')
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    source = PythonLaunchDescriptionSource(path)
    launch_description.add_action(
        IncludeLaunchDescription(source, launch_arguments=args.items())
    )


def add_keyboard_control_node(launch_description: LaunchDescription) -> None:
    action = Node(
        executable='keyboard_control_node',
        package='keyboard_control',
        name='keyboard_control',
        namespace=LaunchConfiguration('vehicle_name'),
        on_exit=Shutdown(),
    )
    launch_description.add_action(action)


################################################################################
# This is the only function strictly required by the ros launch system. It has
# to be named `generate_launch_description` and has to return a
# `LaunchDescription` object. All other functions are just custom convenience
# functions to make the code more structured and readable.
################################################################################
def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    include_simulation_launch(launch_description=launch_description)
    add_keyboard_control_node(launch_description=launch_description)
    return launch_description
