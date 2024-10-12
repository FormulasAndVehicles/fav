from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def declare_launch_args(launch_description: LaunchDescription) -> None:
    declare_vehicle_name_and_sim_time(
        launch_description=launch_description, use_sim_time_default='true'
    )

    action = DeclareLaunchArgument(
        'start_gui',
        default_value='true',
        description=(
            'Start the gazebo GUI. '
            'Otherwise gazebo will run in headless mode.'
        ),
    )
    launch_description.add_action(action)


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    ############################################################################
    # GAZEBO
    ############################################################################
    package_path = get_package_share_path('hippo_sim')
    path = str(package_path / 'launch/start_gazebo.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add('start_gui')
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)

    ############################################################################
    # SPAWN BLUEROV
    ############################################################################
    package_path = get_package_share_path('hippo_sim')
    path = str(package_path / 'launch/spawn_bluerov.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)

    return launch_description
