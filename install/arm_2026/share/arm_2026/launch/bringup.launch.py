from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("arm_2026")

    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_share, "description", "arm_2026.urdf.xacro"])
    ])

    controller_config = PathJoinSubstitution(
        [pkg_share, "config", "controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controller_config
        ],
        output="screen"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    position_controller_spawner = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "position_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher,
        rviz_node,
        joint_state_broadcaster_spawner,
        position_controller_spawner
    ])
