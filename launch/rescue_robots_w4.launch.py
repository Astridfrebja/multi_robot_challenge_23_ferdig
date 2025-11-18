import os
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_robot_entities(context, package_name, world_file):
    """Create spawn + controller nodes for the requested number of robots."""
    actions = []

    try:
        robot_count = int(LaunchConfiguration('robot_count').perform(context))
    except (ValueError, TypeError):
        robot_count = 2

    if robot_count <= 0:
        return actions

    def _as_float(name, default):
        try:
            return float(LaunchConfiguration(name).perform(context))
        except (ValueError, TypeError):
            return default

    spacing = _as_float('spawn_spacing', 2.0)
    origin_x = _as_float('spawn_origin_x', -0.5)
    origin_y = _as_float('spawn_origin_y', -0.3)
    spawn_yaw = _as_float('spawn_yaw', 0.0)

    use_sim_time_raw = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time_bool = str(use_sim_time_raw).lower() in ('true', '1', 'yes')
    use_sim_time_str = 'true' if use_sim_time_bool else 'false'

    yaw_str = f'{spawn_yaw:.6f}'

    for idx in range(robot_count):
        namespace = f'tb3_{idx}'
        offset = idx - (robot_count - 1) / 2.0
        x_val = origin_x
        y_val = origin_y + offset * spacing
        spawn_launch = PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch'), '/spawn_robot.launch.py']
        )

        actions.append(
            IncludeLaunchDescription(
                spawn_launch,
                launch_arguments={
                    'namespace': namespace,
                    'x': f'{x_val:.3f}',
                    'y': f'{y_val:.3f}',
                    'yaw': yaw_str,
                    'use_sim_time': use_sim_time_str,
                }.items(),
            )
        )

        actions.append(
            Node(
                package=package_name,
                executable='robot_node',
                name=f'robot_controller_{namespace}',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time_bool,
                    'world_file': world_file,
                }],
            )
        )

    return actions


def generate_launch_description():
    package_name = 'multi_robot_challenge_23'

    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'dat160_w4.world')
    map_file_path = os.path.join(get_package_share_directory(package_name), 'maps', 'map_dat160_w4.yaml')
    rviz_config_file_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'model.rviz')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='2',
        description='Number of TurtleBots to spawn in the scenario'
    )
    spawn_spacing_arg = DeclareLaunchArgument(
        'spawn_spacing',
        default_value='2.0',
        description='Spacing in meters between spawned robots along the Y-axis'
    )
    spawn_origin_x_arg = DeclareLaunchArgument(
        'spawn_origin_x',
        default_value='-0.5',
        description='Base X position for spawned robots'
    )
    spawn_origin_y_arg = DeclareLaunchArgument(
        'spawn_origin_y',
        default_value='-0.3',
        description='Center Y position for spawned robots'
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='Yaw (radians) applied to all spawned robots'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Starting Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path}.items()
    )

    # Starting Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{"yaml_filename": map_file_path, "topic_name": "map", "frame_id": "map"}],
    )

    # Starting a lifecycle manager 
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ["map_server"]}]
    )

    # Starting rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    # Scoring system node
    scoring_node = Node(
        package='scoring',
        executable='scoring',
        name='scoring',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        sim_time_arg,
        robot_count_arg,
        spawn_spacing_arg,
        spawn_origin_x_arg,
        spawn_origin_y_arg,
        spawn_yaw_arg,
        gazebo,
        map_server,
        lifecycle_manager,
        rviz_node,
        scoring_node,
        OpaqueFunction(function=partial(_spawn_robot_entities, package_name=package_name, world_file=world_file_path)),
    ])