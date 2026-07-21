from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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

    joystick_teleop = Node(
        package="arm_control",
        executable="joystick_teleop",
        name="arm_2026_joystick_teleop",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription([
        joy_node,
        joystick_teleop,
        rviz_node,
    ])
