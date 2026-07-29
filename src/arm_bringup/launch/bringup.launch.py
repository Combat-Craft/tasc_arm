from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("arm_description")
    control_share = FindPackageShare("arm_control")

    urdf_file = PathJoinSubstitution([
        description_share,
        "urdf",
        "arm_2026.urdf.xacro",
    ])

    controller_config = PathJoinSubstitution([
        control_share,
        "config",
        "controllers.yaml",
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            urdf_file,
        ]),
        value_type=str,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="arm",
        parameters=[
            {
                "robot_description": robot_description,
            },
            controller_config,
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="arm",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
        remappings=[
            ("/joint_states", "/arm/joint_states"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
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
        arguments=[
            "manual_controller",
            "--controller-manager",
            "/arm/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        namespace="arm",
        name="joy_node",
        parameters=[{
            "dev": "/dev/input/js0",
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
        }],
        output="screen",
    )

    joystick_teleop_node = Node(
        package="arm_control",
        executable="joystick_teleop",
        namespace="arm",
        name="joystick_teleop",
        parameters=[{
        "joy_topic": "/arm/joy",
        "command_topic": "/arm/manual_controller/commands",
        }],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="arm",
        remappings=[
            ("/joint_states", "/arm/joint_states"),
        ],
        output="screen",
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,

        # Hardware attachment can take several seconds.
        TimerAction(
            period=7.0,
            actions=[
                joint_state_broadcaster_spawner,
            ],
        ),

        TimerAction(
            period=9.0,
            actions=[
                manual_controller_spawner,
            ],
        ),

        joy_node,
        joystick_teleop_node,
        rviz_node,
    ])
