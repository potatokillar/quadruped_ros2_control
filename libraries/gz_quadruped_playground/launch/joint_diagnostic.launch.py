from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def spawner(name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager"],
    )


def generate_launch_description():
    joint_state_broadcaster = spawner("joint_state_broadcaster")
    diagnostic_controllers = [
        spawner("rl_hip_diagnostic_controller"),
        spawner("rl_thigh_diagnostic_controller"),
        spawner("rl_calf_diagnostic_controller"),
    ]

    return LaunchDescription([
        joint_state_broadcaster,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=diagnostic_controllers,
            )
        ),
    ])
