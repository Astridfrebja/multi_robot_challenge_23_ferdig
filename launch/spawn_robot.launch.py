import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


package_name_robot = 'multi_robot_challenge_23'

def get_robot_description(context):
    xacro_file_name = 'turtlebot3_waffle_pi.urdf.xacro'
    xacro_file_path = os.path.join(get_package_share_directory(package_name_robot), 'urdf', xacro_file_name)
    mesh_dir = os.path.join(get_package_share_directory('turtlebot3_description'),'meshes')

    robot_description = xacro.process_file(
        xacro_file_path, 
        mappings={
            "mesh_dir": mesh_dir,
            "namespace": context.launch_configurations['namespace'],
        },
    ).toxml()

    return [SetLaunchConfiguration('robot_desc', robot_description)]

def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='tb3_5'
    )
    robot_x_pos_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0'
    )
    robot_y_pos_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0'
    )
    robot_yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_arg = OpaqueFunction(function=get_robot_description)
    robot_description = LaunchConfiguration('robot_desc')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'frame_prefix': [namespace,'/'],        
            'use_sim_time': use_sim_time
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        namespace=namespace,
        arguments=['-topic', 'robot_description',
                    '-entity', namespace,
                    '-robot_namespace', namespace,
                    '-x', x, '-y', y, '-Y', yaw],
        output='screen'
    )
    
    odom_topic = [namespace,'/odom']
    tf_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', odom_topic],
    )

    aruco_recognition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name_robot), 'launch'), '/aruco_recognition.launch.py']),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )

    return LaunchDescription([
        namespace_launch_arg,
        use_sim_time_arg,
        robot_description_arg,
        robot_x_pos_arg,
        robot_y_pos_arg,
        robot_yaw_arg,
        robot_state_publisher,
        spawn_entity,
        tf_map_to_odom,
        aruco_recognition,
    ])
