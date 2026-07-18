from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction


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
        parameters=[
            {"robot_description": robot_description_content},
            controller_config,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
    	package="robot_state_publisher",
    	executable="robot_state_publisher",
    	parameters=[
    	    {"robot_description": robot_description_content}
    	],
    	output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "dev": "/dev/input/js0",
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
        }],
        output="screen",
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    manual_controller_spawner = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "manual_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    delayed_controller_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            manual_controller_spawner
        ]
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher,
        rviz_node,
        joy_node,
        delayed_controller_spawners
    ])
