# ros2_learning_panda_gazebo_demo

Beginner-friendly Panda Gazebo Sim + RViz2 demo with a fixed camera (not on the arm). The Panda moves in Gazebo, and RViz2 follows via shared joint states.

## Features
- Gazebo Sim world + fixed camera
- Panda joint position controllers driven by a simple sinusoidal node
- `joint_states` bridged from Gazebo to ROS, RViz2 synced by `robot_state_publisher`
- RViz2 includes camera image display

## Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-moveit-resources-panda-description \
  ros-jazzy-rviz2
```

## Build
```bash
cd ~/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_panda_gazebo_demo
source install/setup.bash
```

## Run
```bash
ros2 launch ros2_learning_panda_gazebo_demo panda_gazebo_rviz.launch.py
```

## Topics
- Joint command (ROS -> Gazebo): `/panda/joint1/cmd_pos` ... `/panda/joint7/cmd_pos`
- Joint state (Gazebo -> ROS): `/joint_states`
- Camera image (Gazebo -> ROS): `/demo_camera/image`

## Layout
```
ros2_learning_panda_gazebo_demo/
├── launch/
├── config/
├── models/
├── urdf/
├── worlds/
└── rviz/
```

## Notes
- The Panda URDF is based on `moveit_resources_panda_description`, with inertial blocks added for Gazebo Sim.
- Motion is a simple sinusoidal trajectory for learning and visualization.
