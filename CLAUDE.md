# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Communication

与用户沟通时尽量使用中文回复。进行代码评审时保持专业，但要极端严格、直指问题本身并提供尖锐的技术批评。

## Repository Layout

```
Ros2Learning/
├── ros2_ws/               # ROS 2 colcon workspace (主体)
│   ├── src/               # ROS 2 packages
│   ├── scripts/           # build/setup/source helper scripts
│   ├── build/             # colcon build artifacts (generated)
│   ├── install/           # colcon install artifacts (generated)
│   └── log/               # colcon logs (generated)
└── cpp_vscode_lab/        # Standalone C++ learning area (no ROS, CMake only)
```

All ROS 2 development lives under `ros2_ws/`. The `cpp_vscode_lab/` directory is fully decoupled and uses plain CMake + gdb.

## Build & Environment Commands

Run from repo root (`/root/Ros2Learning`):

```bash
# One-time: install system deps + rosdep (requires sudo)
./ros2_ws/scripts/setup_ubuntu.sh jazzy

# Build entire workspace
./ros2_ws/scripts/build.sh jazzy

# Source ROS 2 + workspace overlay in current shell (must use `source`, not execute)
source ./ros2_ws/scripts/source.sh jazzy

# Build a single package (run from inside ros2_ws/ after sourcing ROS base)
cd ros2_ws && colcon build --packages-select <pkg_name> --symlink-install

# Run tests (if any exist in a package)
cd ros2_ws && colcon test --packages-select <pkg_name>
colcon test-result --verbose
```

The `build.sh` script runs `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` and merges all per-package `compile_commands.json` into `ros2_ws/compile_commands.json`.

For the standalone C++ lab:
```bash
cd cpp_vscode_lab
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

## Running Packages

After sourcing the workspace:

```bash
# Basic pub/sub sanity check
ros2 run ros2_learning_cpp talker
ros2 run ros2_learning_cpp listener

# QoS demo (Composition mode – both nodes in one process)
ros2 launch ros2_learning_qos qos_demo.launch.py

# BehaviorTree demo (mock mode, no Nav2 needed)
ros2 launch ros2_learning_behavior_tree bt_demo.launch.py tree_file:=mock_fallback_demo.xml

# Multi-threaded executor comparison
ros2 launch ros2_learning_multithreading multi_threaded.launch.py

# Task-driven navigation (requires Nav2 running)
ros2 launch ros2_learning_task_runner task_runner_sim.launch.py
```

## Architecture Overview

### ROS 2 Packages (`ros2_ws/src/`)

Each package is an independent learning module. Key packages:

| Package | Purpose |
|---|---|
| `ros2_learning_cpp` | Basic pub/sub (talker/listener), node composition, lifecycle nodes |
| `ros2_learning_qos` | QoS strategies (Best Effort, Reliable, Transient Local, Deadline) using a Mars rover simulation; uses **Composition** architecture (both rover + mission_control run in one container) |
| `ros2_learning_behavior_tree` | BehaviorTree.CPP v4 integration; supports **mock mode** (no Nav2) and **real mode** (requires Nav2 `/navigate_to_pose` action) |
| `ros2_learning_multithreading` | SingleThreadedExecutor vs MultiThreadedExecutor; MutuallyExclusive vs Reentrant CallbackGroups |
| `ros2_learning_kinematics` | 2-DOF planar IK (geometric method) and 6-DOF spherical wrist FK/IK (D-H parameters, kinematic decoupling) |
| `ros2_learning_task_runner` | Task orchestration: reads `task_plan.yaml`, navigates to pickup points via `navigate_to_pose` action, calls pick/place services |
| `ros2_learning_manipulation_stub` | Stub services `/manipulation/pick` and `/manipulation/place` (`std_srvs/Trigger`) for task_runner integration |
| `ros2_learning_nav2_client` | Parameterized Nav2 client (goal pose, TF frames, sim time) |
| `ros2_learning_tb3_spawner` | Service-based TurtleBot3 spawner for Gazebo |
| `ros2_learning_turtlebot3_teleop` | Keyboard teleop for TurtleBot3 |
| `ros2_learning_turtlesim_go_to` | Turtlesim proportional control demo |
| `ros2_learning_sysinfo_publisher` | System info publisher |
| `ros2_learning_sysinfo_qt_viewer` | Qt-based system info viewer |
| `ros2_learning_yolo_detector` | YOLO object detection integration |
| `ros2_learning_panda_gazebo_demo` | Panda arm + Gazebo Sim + RViz2 demo |
| `ros2_learning_tf_quaternion_demo` | TF and quaternion usage demo |
| `ros2_learning_arm_basics` | Arm control basics |
| `ros2_learning_control_demo` | ros2_control demo |

### Navigation Stack Integration

The task-driven navigation architecture:
- `task_runner` reads `config/task_plan.yaml` (pickup coords + dropoff coord)
- Calls `navigate_to_pose` Nav2 action → publishes feedback on `/distance_remaining`
- Calls `/manipulation/pick` and `/manipulation/place` services from `manipulation_stub`
- Requires: Gazebo (terminal A) → Nav2 (terminal B) → task_runner (terminal C)

### BehaviorTree.CPP v4 Pattern

XML tree files live in `ros2_learning_behavior_tree/behavior_trees/`. Custom BT nodes are in `src/nodes/`. Nodes requiring ROS context (e.g., action clients) are registered via `registerBuilder` with a lambda capturing the `rclcpp::Node::SharedPtr`.

### QoS / Composition Pattern

`ros2_learning_qos` registers both `RoverNode` and `MissionControlNode` as rclcpp components (plugins) via `RCLCPP_COMPONENTS_REGISTER_NODE`. The launch file uses `ComposableNodeContainer` to run them in-process for zero-copy communication.

## Header Organization

Most packages use `include/<package_name>/` with `#include "<package_name>/foo.hpp"`. Some older or simpler packages use flat `include/` with `#include "foo.hpp"`. If naming conflicts arise across packages, switch to the namespaced form.

## Code Style

- **Bash scripts**: `set -euo pipefail`, 2-space indent, matching `ros2_ws/scripts/*.sh` style
- **C++**: 4-space indent
- **YAML/launch**: 2-space indent
- **Package names**: lowercase with underscores (e.g., `ros2_learning_cpp`)

## VS Code Debugging (rclcpp Step-Through)

First-time setup on a new machine:
```bash
./ros2_ws/scripts/setup_rclcpp_source.sh jazzy   # rclcpp source for step-into
./ros2_ws/scripts/setup_ros2_dbgsym.sh jazzy      # dbgsym packages (stripped by default)
./ros2_ws/scripts/setup_glibc_source.sh           # optional: step into libc/_start
./ros2_ws/scripts/setup_rcutils_source.sh jazzy   # optional: step into rcutils
```

The `.vscode/` config includes `sourceFileMap` entries mapping system paths to local source directories.

## Environment

- **ROS distro**: Jazzy (`/opt/ros/jazzy`)
- **OS**: Ubuntu 24.04 (WSL2)
- **Simulator**: Gazebo Sim (Harmonic) via `ros_gz_sim`
- **Robot**: TurtleBot3 Waffle Pi (`export TURTLEBOT3_MODEL=waffle_pi`)
- **No tests exist yet** in this repo; new packages should add tests using ament/colcon conventions named `test_*` or `*_test`
