#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace
{
class TerminalRawMode
{
public:
  TerminalRawMode() = default;
  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode & operator=(const TerminalRawMode &) = delete;

  bool enable()
  {
    if (!isatty(STDIN_FILENO)) {
      return false;
    }

    if (tcgetattr(STDIN_FILENO, &old_) != 0) {
      return false;
    }

    termios raw = old_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      return false;
    }

    enabled_ = true;
    return true;
  }

  void disable()
  {
    if (!enabled_) {
      return;
    }
    (void)tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    enabled_ = false;
  }

  ~TerminalRawMode() { disable(); }

private:
  termios old_{};
  bool enabled_{false};
};

int read_key_nonblocking()
{
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;

  const int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
  if (ret <= 0) {
    return -1;
  }

  char c = 0;
  const ssize_t n = ::read(STDIN_FILENO, &c, 1);
  if (n <= 0) {
    return -1;
  }
  return static_cast<unsigned char>(c);
}

std::string help_text()
{
  return std::string(
    "Keyboard Teleop (C++)\n"
    "-------------------\n"
    "w/s : forward/back\n"
    "a/d : turn left/right\n"
    "x or space : stop\n"
    "q/e : increase/decrease both speeds\n"
    "r/f : increase/decrease linear speed\n"
    "t/g : increase/decrease angular speed\n"
    "?   : show this help\n"
    "CTRL-C to quit\n");
}

}  // namespace

class TeleopKeyboardNode : public rclcpp::Node
{
public:
  TeleopKeyboardNode()
  : rclcpp::Node("ros2_learning_teleop_keyboard")
  {
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("linear_scale", 0.22);
    this->declare_parameter<double>("angular_scale", 2.84);
    this->declare_parameter<double>("publish_rate_hz", 20.0);
    this->declare_parameter<double>("command_timeout_s", 0.5);

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    command_timeout_s_ = this->get_parameter("command_timeout_s").as_double();

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 20.0;
    }

    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "%s", help_text().c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing to %s (linear_scale=%.3f angular_scale=%.3f)",
      cmd_vel_topic_.c_str(), linear_scale_, angular_scale_);

    if (!raw_.enable()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to enable terminal raw mode (is this a TTY?). Keyboard input may not work.");
    }

    last_cmd_time_ = this->now();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TeleopKeyboardNode::on_timer, this));
  }

private:
  void on_timer()
  {
    process_keyboard();

    const auto now = this->now();
    const double age = (now - last_cmd_time_).seconds();
    if (age > command_timeout_s_) {
      geometry_msgs::msg::TwistStamped stop;
      stop.header.stamp = now;
      pub_->publish(stop);
      return;
    }

    current_cmd_.header.stamp = now;
    pub_->publish(current_cmd_);
  }

  void process_keyboard()
  {
    for (;;) {
      const int key = read_key_nonblocking();
      if (key < 0) {
        break;
      }

      bool updated_cmd = false;
      bool updated_scale = false;

      switch (key) {
        case 'w':
        case 'W':
          current_cmd_.twist.linear.x = linear_scale_;
          current_cmd_.twist.angular.z = 0.0;
          updated_cmd = true;
          break;
        case 's':
        case 'S':
          current_cmd_.twist.linear.x = -linear_scale_;
          current_cmd_.twist.angular.z = 0.0;
          updated_cmd = true;
          break;
        case 'a':
        case 'A':
          current_cmd_.twist.linear.x = 0.0;
          current_cmd_.twist.angular.z = angular_scale_;
          updated_cmd = true;
          break;
        case 'd':
        case 'D':
          current_cmd_.twist.linear.x = 0.0;
          current_cmd_.twist.angular.z = -angular_scale_;
          updated_cmd = true;
          break;
        case 'x':
        case 'X':
        case ' ':
          current_cmd_ = geometry_msgs::msg::TwistStamped{};
          updated_cmd = true;
          break;
        case 'q':
        case 'Q':
          linear_scale_ *= 1.1;
          angular_scale_ *= 1.1;
          updated_scale = true;
          break;
        case 'e':
        case 'E':
          linear_scale_ *= 0.9;
          angular_scale_ *= 0.9;
          updated_scale = true;
          break;
        case 'r':
        case 'R':
          linear_scale_ *= 1.1;
          updated_scale = true;
          break;
        case 'f':
        case 'F':
          linear_scale_ *= 0.9;
          updated_scale = true;
          break;
        case 't':
        case 'T':
          angular_scale_ *= 1.1;
          updated_scale = true;
          break;
        case 'g':
        case 'G':
          angular_scale_ *= 0.9;
          updated_scale = true;
          break;
        case '?':
          RCLCPP_INFO(this->get_logger(), "%s", help_text().c_str());
          break;
        default:
          break;
      }

      if (updated_scale) {
        if (linear_scale_ < 0.0) {
          linear_scale_ = 0.0;
        }
        if (angular_scale_ < 0.0) {
          angular_scale_ = 0.0;
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Updated scales: linear_scale=%.3f angular_scale=%.3f",
          linear_scale_, angular_scale_);
      }

      if (updated_cmd || updated_scale) {
        last_cmd_time_ = this->now();
      }
    }
  }

  TerminalRawMode raw_;

  std::string cmd_vel_topic_;
  double linear_scale_{0.22};
  double angular_scale_{2.84};
  double publish_rate_hz_{20.0};
  double command_timeout_s_{0.5};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::TwistStamped current_cmd_{};
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopKeyboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
