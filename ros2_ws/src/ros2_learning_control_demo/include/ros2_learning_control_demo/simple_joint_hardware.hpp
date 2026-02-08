// Copyright 2026 user
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file simple_joint_hardware.hpp
 * @brief 极简的 ros2_control Hardware Interface 示例
 *
 * 这个文件定义了一个虚拟的单关节硬件接口。
 * 它不连接任何真实硬件，只是在内存中模拟关节状态。
 *
 * 学习要点:
 * =========
 * 1. HardwareInterface 是 ros2_control 与硬件交互的抽象层
 * 2. 你需要实现 on_init(), export_state_interfaces(),
 *    export_command_interfaces(), read(), write() 等方法
 * 3. StateInterface: 向控制器提供硬件状态（如位置、速度）
 * 4. CommandInterface: 接收控制器的命令（如目标位置）
 */

#ifndef ROS2_LEARNING_CONTROL_DEMO__SIMPLE_JOINT_HARDWARE_HPP_
#define ROS2_LEARNING_CONTROL_DEMO__SIMPLE_JOINT_HARDWARE_HPP_

#include <string>
#include <vector>

// ============================================================
// ros2_control 核心头文件
// ============================================================

// SystemInterface: Hardware Interface 的基类
// 你需要继承这个类来实现自己的硬件接口
#include "hardware_interface/system_interface.hpp"

// Handle: 定义 StateInterface 和 CommandInterface
// 用于在控制器和硬件之间传递数据
#include "hardware_interface/handle.hpp"

// HardwareInfo: 从 URDF 解析出的硬件配置信息
// 包含关节名称、接口类型等
#include "hardware_interface/hardware_info.hpp"

// return_type: read() 和 write() 函数的返回值类型
// OK 表示成功，ERROR 表示失败
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ============================================================
// ROS 2 通用头文件
// ============================================================

// RCLCPP_SHARED_PTR_DEFINITIONS 宏的定义
// 用于自动生成 SharedPtr、WeakPtr 等类型别名
#include "rclcpp/macros.hpp"

// State: 生命周期状态类
// 用于 on_activate/on_deactivate 等回调函数的参数
#include "rclcpp_lifecycle/state.hpp"

namespace ros2_learning_control_demo
{

/**
 * @class SimpleJointHardware
 * @brief 虚拟单关节硬件接口
 *
 * 这个类模拟一个简单的关节:
 * - 有一个位置状态 (position state)
 * - 有一个位置命令 (position command)
 * - read() 时，模拟关节逐渐移动到目标位置
 * - write() 时，什么都不做（因为是虚拟硬件）
 */
class SimpleJointHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SimpleJointHardware)

  // ============================================================
  // 生命周期回调函数
  // ============================================================

  /**
   * @brief 初始化硬件接口
   *
   * 在这里解析 URDF 中的 <ros2_control> 配置，
   * 初始化内部变量。
   *
   * @param info 从 URDF 解析出的硬件信息
   * @return 初始化是否成功
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  // ============================================================
  // 接口导出
  // ============================================================

  /**
   * @brief 导出状态接口
   *
   * 告诉 ros2_control 这个硬件可以提供哪些状态数据。
   * 例如: joint1/position, joint1/velocity
   *
   * @return 状态接口列表
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief 导出命令接口
   *
   * 告诉 ros2_control 这个硬件可以接收哪些命令。
   * 例如: joint1/position (目标位置)
   *
   * @return 命令接口列表
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ============================================================
  // 激活/停用回调
  // ============================================================

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ============================================================
  // 核心读写函数
  // ============================================================

  /**
   * @brief 从硬件读取状态
   *
   * 在真实硬件中，这里会从编码器、传感器读取数据。
   * 在这个虚拟硬件中，我们模拟关节逐渐移动到目标位置。
   *
   * @param time 当前时间
   * @param period 距上次调用的时间间隔
   * @return 读取是否成功
   */
  hardware_interface::return_type read(const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief 向硬件写入命令
   *
   * 在真实硬件中，这里会向电机驱动器发送控制指令。
   * 在这个虚拟硬件中，我们什么都不做（位置命令已经在 read() 中使用了）。
   *
   * @param time 当前时间
   * @param period 距上次调用的时间间隔
   * @return 写入是否成功
   */
  hardware_interface::return_type write(const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // 关节名称
  std::string m_jointName;

  // 关节状态（当前位置）
  double m_position = 0.0;

  // 关节命令（目标位置）
  double m_positionCommand = 0.0;
};

}  // namespace ros2_learning_control_demo

#endif  // ROS2_LEARNING_CONTROL_DEMO__SIMPLE_JOINT_HARDWARE_HPP_
