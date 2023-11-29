from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def declare_launch_args(launch_description: LaunchDescription) -> None:
    declare_vehicle_name_and_sim_time(launch_description=launch_description,
                                      use_sim_time_default='true')

    package_path = get_package_share_path('hippo_control')
    default_path = str(package_path /
                       'config/actuator_mixer/bluerov_normalized_default.yaml')
    action = DeclareLaunchArgument(
        name='mixer_path',
        default_value=default_path,
        description='Path to mixer configuration .yaml file')
    launch_description.add_action(action)

    action = DeclareLaunchArgument(
        'start_gui',
        default_value='true',
        description=
        'Start the gazebo GUI. Otherwise gazebo will run in headless mode.')
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
    # SPAWN APRILTAGS
    ############################################################################
    path = str(package_path / 'launch/spawn_wall_tags.launch.py')
    source = PythonLaunchDescriptionSource(path)
    action = IncludeLaunchDescription(source)
    launch_description.add_action(action)

    ############################################################################
    # START MIXER
    ############################################################################
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    package_path = get_package_share_path('hippo_control')
    path = str(package_path /
               'config/actuator_mixer/bluerov_normalized_default.yaml')
    args['mixer_path'] = path
    path = str(package_path / 'launch/node_actuator_mixer.launch.py')
    source = PythonLaunchDescriptionSource(path)
    mixer = IncludeLaunchDescription(source, launch_arguments=args.items())
    action = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        mixer,
    ])
    launch_description.add_action(action)

    ############################################################################
    # SPAWN BLUEROV
    ############################################################################
    package_path = get_package_share_path('hippo_sim')
    path = str(package_path / 'launch/spawn_bluerov.launch.py')
    source = PythonLaunchDescriptionSource(path)
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args['use_range_sensor'] = 'true'
    action = IncludeLaunchDescription(source, launch_arguments=args.items())
    launch_description.add_action(action)

    return launch_description
