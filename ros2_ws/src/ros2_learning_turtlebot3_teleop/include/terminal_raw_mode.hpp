#pragma once

// terminal_raw_mode.hpp
// - 终端 raw mode 的“声明”部分。
// - 通过 PImpl 把 termios 等平台相关细节隐藏在 .cpp 中，避免污染其它编译单元。
//   这样做的收益：
//   - 头文件更轻量，减少编译依赖；
//   - 与系统相关的 include（termios/unistd）不会传播到整个工程；
//   - 调试时更清晰：raw-mode 的断点都集中在 terminal_raw_mode.cpp。
//
// 典型用途：键盘遥控场景下希望“按键即响应”，不需要回车。

#include <memory>

namespace ros2_learning_turtlebot3_teleop::detail
{

class TerminalRawMode
{
  public:
    TerminalRawMode();
    TerminalRawMode(const TerminalRawMode &) = delete;
    TerminalRawMode &operator=(const TerminalRawMode &) = delete;

    // 尝试把 STDIN 设置为 raw mode。
    // 返回 false 代表：不是 TTY（例如 launch 重定向/非交互终端）、或 termios 调用失败。
    // 约定：enable() 失败时，调用方可以选择继续运行（只是不支持键盘输入），便于排障。
    bool enable();

    // 如果曾经 enable() 成功，则恢复到原有终端模式。
    // 该函数可重复调用（重复调用会被安全地忽略）。
    void disable();

    ~TerminalRawMode();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool enabled_;
};

} // namespace ros2_learning_turtlebot3_teleop::detail
