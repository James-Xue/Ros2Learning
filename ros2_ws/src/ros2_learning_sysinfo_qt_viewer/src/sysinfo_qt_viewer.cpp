#include <memory>
#include <string>

#include <QApplication>
#include <QMainWindow>
#include <QTextEdit>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// 这个程序把 ROS2 的 spin_some() 嵌到 Qt 事件循环里。
// 设计意图：
// - 不额外创建线程，避免 UI 线程/ROS 线程切换的同步复杂度。
// - 通过一个 QTimer 周期调用 executor.spin_some()，处理订阅回调。

class SysInfoQtNode : public rclcpp::Node
{
  public:
    explicit SysInfoQtNode(QTextEdit *view)
            : rclcpp::Node("ros2_learning_sysinfo_qt_viewer")
            , view_(view)
    {
        topic_ = this->declare_parameter<std::string>("topic",
                                                      "/ros2_learning/sysinfo");

        sub_ = this->create_subscription<std_msgs::msg::String>(
            topic_, 10,
            [this](const std_msgs::msg::String &msg)
            {
                // 回调运行在 Qt 主线程（因为我们在 Qt 的 timer 里 spin_some）。
                // 所以可以直接更新 UI。
                if (view_)
                {
                    view_->setPlainText(QString::fromStdString(msg.data));
                }
            });

        if (view_)
        {
            view_->setPlainText(
                QString("Subscribed to %1\nWaiting for messages...")
                    .arg(QString::fromStdString(topic_)));
        }
    }

  private:
    std::string topic_;
    QTextEdit *view_{nullptr};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    // Qt 会解析并移除它认识的参数；剩余参数仍可供 rclcpp 使用。
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    QMainWindow window;
    window.setWindowTitle("ROS2 SysInfo Viewer (Qt)");

    auto *text = new QTextEdit();
    text->setReadOnly(true);
    window.setCentralWidget(text);
    window.resize(900, 600);
    window.show();

    auto node = std::make_shared<SysInfoQtNode>(text);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // 以较高频率处理 ROS 回调；UI 更新频率取决于发布端 publish_rate_hz。
    QTimer ros_spin_timer;
    QObject::connect(&ros_spin_timer, &QTimer::timeout,
                     [&exec]() { exec.spin_some(); });
    ros_spin_timer.start(10);

    const int ret = app.exec();

    exec.remove_node(node);
    rclcpp::shutdown();
    return ret;
}
