# tasc_arm

ROS 2 workspace for the TASC robotic arm. This repo currently contains:
- **URDF** for RViz visualization (runs in parallel)
- **ros2_control** setup (controllers + hardware plugin scaffold)
- **Teleop** (keyboard and Logitech Extreme 3D Pro Joystick) for joint control
- **Bringup launch** to start RViz, robot_state_publisher, ros2_control, and joystick nodes

## Repo layout
```bash
tasc_arm/ 
src/ 
arm_description/ # URDF 
arm_control/ # ros2_control configs + hardware plugin + teleop nodes 
arm_bringup/ # launch files (RViz + control bringup)
```
### Packages 

#### `arm_description`
- Contains the robot model urdf used by RViz and robot_state_publisher.

#### `arm_control`
- Contains controller config + teleop nodes.
- Also includes a **ros2_control hardware interface plugin** scaffold:

#### `arm_bringup`
- Launch files to bring up the full stack.
- Key file:
  - `src/arm_bringup/launch/bringup.launch.py`

## Dependencies

Assuming Ubuntu 22.04 + ROS 2 Humble.

Install common deps:
```bash
sudo apt update
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-forward-command-controller \
  ros-humble-joy
```
Python deps (for IK teleop):
```bash
python3 -m pip install --user ikpy numpy
```

## Build
From the workspace root:
```bash
cd ~/tasc_arm
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run
### Arm bringup (RViz + robot_state_publisher + joy_node + ros2_control + controllers)
```bash
ros2 launch arm_bringup bringup.launch.py
```
### Teleop nodes
#### Keyboard joint teleop
```bash
ros2 run arm_control keyboard_teleop
```
#### Logitech Joystick joint teleop
```bash
ros2 run arm_control joystick_teleop
```

