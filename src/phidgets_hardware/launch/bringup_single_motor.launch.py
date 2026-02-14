from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory("phidgets_hardware")
    xacro_path = os.path.join(pkg_share, "description", "phidgets_single_motor.urdf.xacro")
    controllers_path = os.path.join(pkg_share, "config", "ros2_control_controllers.yaml")

    robot_description = xacro.process_file(xacro_path).toxml()

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_path,
        ],
        output="screen",
    )

    spawn_jsb = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_fwd = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "forward_position_controller", "--controller-manager", "/controller_manager"],
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
        ros2_control_node, 
        spawn_jsb, 
        spawn_fwd,
        joy_node,
    ])

