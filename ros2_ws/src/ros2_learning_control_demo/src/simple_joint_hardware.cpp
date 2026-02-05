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
 * @file simple_joint_hardware.cpp
 * @brief 极简的 ros2_control Hardware Interface 实现
 *
 * 这个文件实现了一个虚拟的单关节硬件接口。
 * 学习这个文件可以帮助你理解:
 * 1. 如何实现一个 HardwareInterface
 * 2. StateInterface 和 CommandInterface 的工作原理
 * 3. read() 和 write() 的执行时机
 */

#include "ros2_learning_control_demo/simple_joint_hardware.hpp"

#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_learning_control_demo
{

// ============================================================
// 全局日志器
// ============================================================
static rclcpp::Logger getLogger()
{
  return rclcpp::get_logger("SimpleJointHardware");
}

// ============================================================
// on_init: 初始化硬件接口
// ============================================================
hardware_interface::CallbackReturn SimpleJointHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 调用父类的 on_init，它会解析 URDF 中的 <ros2_control> 配置
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 检查关节数量（我们的示例只支持一个关节）
  if (info_.joints.size() != 1)
  {
    RCLCPP_ERROR(getLogger(), "SimpleJointHardware 只支持单个关节，但配置了 %zu 个关节",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 获取关节名称
  m_jointName = info_.joints[0].name;

  RCLCPP_INFO(getLogger(), "SimpleJointHardware 初始化成功，关节名: %s", m_jointName.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// export_state_interfaces: 导出状态接口
// ============================================================
std::vector<hardware_interface::StateInterface>
SimpleJointHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 导出位置状态接口
  // 控制器可以通过这个接口读取当前关节位置
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      m_jointName,
      hardware_interface::HW_IF_POSITION,
      &m_position
      )
    );

  RCLCPP_INFO(getLogger(), "导出状态接口: %s/%s",
    m_jointName.c_str(), hardware_interface::HW_IF_POSITION);

  return state_interfaces;
}

// ============================================================
// export_command_interfaces: 导出命令接口
// ============================================================
std::vector<hardware_interface::CommandInterface>
SimpleJointHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // 导出位置命令接口
  // 控制器可以通过这个接口发送目标位置
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      m_jointName,
      hardware_interface::HW_IF_POSITION,
      &m_positionCommand
      )
    );

  RCLCPP_INFO(getLogger(), "导出命令接口: %s/%s",
    m_jointName.c_str(), hardware_interface::HW_IF_POSITION);

  return command_interfaces;
}

// ============================================================
// on_activate: 激活硬件
// ============================================================
hardware_interface::CallbackReturn SimpleJointHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 初始化命令为当前位置（避免突然跳变）
  m_positionCommand = m_position;

  RCLCPP_INFO(getLogger(), "SimpleJointHardware 已激活，当前位置: %.3f", m_position);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate: 停用硬件
// ============================================================
hardware_interface::CallbackReturn SimpleJointHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "SimpleJointHardware 已停用");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// read: 从硬件读取状态
// ============================================================
hardware_interface::return_type SimpleJointHardware::read(const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // 在真实硬件中，这里会从编码器读取当前位置
  // 在虚拟硬件中，我们模拟一个简单的一阶系统：
  // 关节逐渐移动到目标位置

  // 计算时间步长（秒）
  double dt = period.seconds();

  // 一阶低通滤波器，模拟关节运动
  // 时间常数 tau = 0.1 秒（越小越快）
  constexpr double kTau = 0.1;
  double alpha = 1.0 - std::exp(-dt / kTau);

  // 更新当前位置
  m_position += alpha * (m_positionCommand - m_position);

  return hardware_interface::return_type::OK;
}

// ============================================================
// write: 向硬件写入命令
// ============================================================
hardware_interface::return_type SimpleJointHardware::write(const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // 在真实硬件中，这里会向电机驱动器发送控制指令
  // 在虚拟硬件中，我们什么都不做
  // （位置命令已经通过 CommandInterface 直接写入 m_positionCommand 了）

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_learning_control_demo

// ============================================================
// 注册插件
// ============================================================
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_learning_control_demo::SimpleJointHardware,
  hardware_interface::SystemInterface
  )
