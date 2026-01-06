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
    if (!isatty(STDIN_FILENO))
    {
        return false;
    }

    if (!impl_)
    {
        return false;
    }

    if (tcgetattr(STDIN_FILENO, &impl_->old) != 0)
    {
        return false;
    }
    impl_->has_old = true;

    termios raw = impl_->old;
    // 关闭规范模式与回显：这样就能不按回车立刻读到单个字符。
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
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
    disable();
}

} // namespace ros2_learning_turtlebot3_teleop::detail
