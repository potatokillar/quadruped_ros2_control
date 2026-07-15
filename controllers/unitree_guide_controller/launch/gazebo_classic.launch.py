import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def compact_xml(xml):
    root = ET.fromstring(xml)
    for element in root.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        if element.tail is not None and not element.tail.strip():
            element.tail = None
    return ET.tostring(root, encoding='unicode')


def launch_setup(context, *args, **kwargs):

    package_description = context.launch_configurations['pkg_description']
    init_height = context.launch_configurations['height']
    world = context.launch_configurations['world']
    pkg_path = os.path.join(get_package_share_directory(package_description))

    if not world:
        package_world = os.path.join(pkg_path, 'worlds', 'gazebo.world')
        if os.path.isfile(package_world):
            world = package_world

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = compact_xml(
        xacro.process_file(xacro_file, mappings={'GAZEBO': 'true', 'CLASSIC': 'true'}).toxml()
    )

    rviz_config_file = os.path.join(get_package_share_directory(package_description), "config", "visualize_urdf.rviz")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    gazebo_arguments = {"verbose": "false"}
    if world:
        gazebo_arguments["world"] = world

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments=gazebo_arguments.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot", "-z", init_height],
        output="screen",
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_pd_controller",
                   "--controller-manager", "/controller_manager"],
    )

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_guide_controller", "--controller-manager", "/controller_manager"],
    )

    return [
        rviz,
        robot_state_publisher,
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        gazebo,
        spawn_entity,
        leg_pd_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[imu_sensor_broadcaster, joint_state_publisher, unitree_guide_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='go1_description',
        description='package for robot description'
    )

    height = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Init height in simulation'
    )

    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz together with Gazebo'
    )

    world = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Gazebo world file; defaults to worlds/gazebo.world from the description package'
    )

    return LaunchDescription([
        pkg_description,
        height,
        rviz,
        world,
        OpaqueFunction(function=launch_setup),
    ])
