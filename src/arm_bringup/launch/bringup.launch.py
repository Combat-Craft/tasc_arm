from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg1 = get_package_share_directory("arm_description")
    urdf_path = os.path.join(pkg1, "urdf", "mini_arm.urdf")
    
    pkg2 = get_package_share_directory("arm_control")
    controllers_path = os.path.join(pkg2, "config", "controllers.yaml")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_path],
        output="screen",
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_forward_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": "/dev/input/js0",
            "deadzone": 0.05,
            "autorepeat_rate": 50.0,
        }],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        spawner_jsb,
        spawner_arm,
        rviz,
        joy_node,
    ])

