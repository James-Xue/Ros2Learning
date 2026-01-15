// terminal_raw_mode.cpp
// - 终端 raw mode 的实现细节：termios / isatty / tcsetattr。
// - 这里故意不在头文件暴露 termios 相关头文件。

#include "ros2_learning_turtlebot3_teleop/terminal_raw_mode.hpp"

#include <termios.h>
#include <unistd.h>

namespace ros2_learning_turtlebot3_teleop::detail
{

struct TerminalRawMode::Impl
{
    termios old{};
    bool has_old{false};
};

TerminalRawMode::TerminalRawMode()
        : impl_(std::make_unique<Impl>())
        , enabled_(false)
{
}

bool TerminalRawMode::enable()
{
    // 只有在交互 TTY 中 raw mode 才有意义。
    // 例如：被 launch 重定向到日志、或输入来自管道时，isatty 会返回 false。
    if (!isatty(STDIN_FILENO))
    {
        return false;
    }

    if (!impl_)
    {
        return false;
    }

    // 保存旧终端配置，方便 disable() 时恢复。
    if (tcgetattr(STDIN_FILENO, &impl_->old) != 0)
    {
        return false;
    }
    impl_->has_old = true;

    termios raw = impl_->old;
    // 关闭规范模式与回显：
    // - ICANON 关闭后，read() 可以按“字符”返回（无需回车）
    // - ECHO   关闭后，按键不会回显到终端（避免干扰日志/显示）
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    // 设置为真正的“非阻塞读取”风格：
    // - VMIN=0/VTIME=0 允许 read() 在没有输入时立即返回
    // 调用方通常会配合 select() 做探测。
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
    {
        return false;
    }

    enabled_ = true;
    return true;
}

void TerminalRawMode::disable()
{
    // 注意：disable() 只在 enable() 成功后才会尝试恢复。
    if (!enabled_)
    {
        return;
    }

    if (impl_ && impl_->has_old)
    {
        (void)tcsetattr(STDIN_FILENO, TCSANOW, &impl_->old);
    }

    enabled_ = false;
}

TerminalRawMode::~TerminalRawMode()
{
    // RAII：节点退出/异常时尽量恢复终端状态，避免终端留在 raw mode
    // 导致“输入不回显”。
    disable();
}

} // namespace ros2_learning_turtlebot3_teleop::detail
