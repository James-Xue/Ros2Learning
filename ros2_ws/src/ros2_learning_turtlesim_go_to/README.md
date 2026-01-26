# ros2_learning_turtlesim_go_to

A tiny ROS 2 (C++) example package that controls **turtlesim** via a **service**.

- `turtlesim_go_to_server`: provides `/ros2_learning/turtlesim/go_to` (custom service)
- `turtlesim_go_to_client`: calls the above service from CLI

The server drives turtlesim **smoothly** by:
- subscribing `/<turtle_name>/pose`
- publishing `/<turtle_name>/cmd_vel`

## Build

```bash
cd ros2_ws
source ./scripts/source.sh jazzy
colcon build --symlink-install --packages-select ros2_learning_turtlesim_go_to
```

## Run

Terminal 1:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:
```bash
cd ros2_ws
source ./scripts/source.sh jazzy
ros2 run ros2_learning_turtlesim_go_to turtlesim_go_to_server
```

Terminal 3:
```bash
cd ros2_ws
source ./scripts/source.sh jazzy
ros2 run ros2_learning_turtlesim_go_to turtlesim_go_to_client 5.5 5.5 1.57 turtle1
```
