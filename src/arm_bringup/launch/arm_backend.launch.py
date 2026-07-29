from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("arm_description")
    control_share = FindPackageShare("arm_control")

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                description_share,
                "urdf",
                "arm_2026.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    controller_config = PathJoinSubstitution([
        control_share,
        "config",
        "controllers.yaml",
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="arm",
        parameters=[
            {"robot_description": robot_description_content},
            controller_config,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="arm",
        parameters=[
            {"robot_description": robot_description_content},
        ],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="arm",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/arm/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    manual_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="arm",
        arguments=[
            "manual_controller",
            "--controller-manager",
            "/arm/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        manual_controller_spawner,
    ])
