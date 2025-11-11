import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'multi_robot_challenge_23'

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='tb3_extra',
        description='Namespace for the robot to be spawned',
    )
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial X position of the robot in Gazebo world coordinates',
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial Y position of the robot in Gazebo world coordinates',
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw (radians) of the robot in Gazebo world coordinates',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    namespace_cfg = LaunchConfiguration('namespace')
    x_cfg = LaunchConfiguration('x')
    y_cfg = LaunchConfiguration('y')
    yaw_cfg = LaunchConfiguration('yaw')
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch'), '/spawn_robot.launch.py']
        ),
        launch_arguments={
            'namespace': namespace_cfg,
            'x': x_cfg,
            'y': y_cfg,
            'yaw': yaw_cfg,
            'use_sim_time': use_sim_time_cfg,
        }.items(),
    )

    controller_node = Node(
        package=package_name,
        executable='robot_node',
        name='robot_controller',
        namespace=namespace_cfg,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_cfg}],
    )

    return LaunchDescription([
        namespace_arg,
        x_arg,
        y_arg,
        yaw_arg,
        use_sim_time_arg,
        spawn_launch,
        controller_node,
    ])

