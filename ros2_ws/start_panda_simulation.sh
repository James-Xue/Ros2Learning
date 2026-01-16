#!/bin/bash
# Panda机械臂MoveIt仿真启动脚本

echo "============================================="
echo "  启动Panda机械臂MoveIt 2仿真环境"
echo "============================================="
echo ""
echo "说明："
echo "- 这会启动RViz2可视化界面"
echo "- 可以使用交互式标记拖动机械臂"
echo "- 也可以在MotionPlanning面板中设置目标位置"
echo ""

# 确保ROS环境已加载
source /opt/ros/jazzy/setup.bash

# 启动MoveIt 2 demo
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
