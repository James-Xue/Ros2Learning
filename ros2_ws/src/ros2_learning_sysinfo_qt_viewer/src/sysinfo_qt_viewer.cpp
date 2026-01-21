#include <memory>
#include <string>

#include <QApplication>
#include <QDateTime>
#include <QFont>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMainWindow>
#include <QTextEdit>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// 这个程序把 ROS2 的 spin_some() 嵌到 Qt 事件循环里。
// 设计意图：
// - 不额外创建线程，避免 UI 线程/ROS 线程切换的同步复杂度。
// - 通过一个 QTimer 周期调用 executor.spin_some()，处理订阅回调。

namespace
{
QString to_pretty_view(const QString &raw)
{
    // 目标：
    // 1) 如果是 JSON：用 Qt 解析后缩进打印，便于肉眼阅读；
    // 2) 同时输出“摘要区”，把关键信息提取出来（更像仪表盘）。
    // 3) 如果解析失败：回退显示原文，避免 UI 空白。

    const QByteArray bytes = raw.toUtf8();
    QJsonParseError err{};
    const QJsonDocument doc = QJsonDocument::fromJson(bytes, &err);

    const QString now = QDateTime::currentDateTime().toString(Qt::ISODate);

    if (err.error != QJsonParseError::NoError || doc.isNull())
    {
        return QString("[Received @ %1]\n\n[Raw]\n%2\n\n[JSON parse error]\n%3 "
                       "(offset=%4)")
            .arg(now, raw, err.errorString())
            .arg(err.offset);
    }

    if (!doc.isObject())
    {
        return QString("[Received @ %1]\n\n[JSON]\n%2")
            .arg(now, QString::fromUtf8(doc.toJson(QJsonDocument::Indented)));
    }

    const QJsonObject root = doc.object();

    const QString hostname = root.value("hostname").toString();
    const QString os = root.value("os").toString();
    const QString kernel = root.value("kernel").toString();
    const QString uptime_s =
        root.value("uptime_s").isDouble()
            ? QString::number(root.value("uptime_s").toDouble())
            : QString();

    const QJsonObject cpu = root.value("cpu").toObject();
    const QString cpu_model = cpu.value("model").toString();
    const int cpu_cores = cpu.value("logical_cores").toInt();
    const QString loadavg = cpu.value("loadavg").toString();

    const QJsonObject mem = root.value("memory_kb").toObject();
    const qint64 mem_total_kb =
        static_cast<qint64>(mem.value("total").toDouble());
    const qint64 mem_avail_kb =
        static_cast<qint64>(mem.value("available").toDouble());

    const QJsonArray net = root.value("net").toArray();

    QString summary;
    summary += QString("[Received @ %1]\n\n").arg(now);
    summary += "=== Summary ===\n";
    summary += QString("Host      : %1\n")
                   .arg(hostname.isEmpty() ? "(unknown)" : hostname);
    summary += QString("OS        : %1\n").arg(os.isEmpty() ? "(unknown)" : os);
    summary += QString("Kernel    : %1\n")
                   .arg(kernel.isEmpty() ? "(unknown)" : kernel);
    summary += QString("Uptime(s) : %1\n\n")
                   .arg(uptime_s.isEmpty() ? "(unknown)" : uptime_s);

    summary += "CPU\n";
    summary += QString("  Model : %1\n")
                   .arg(cpu_model.isEmpty() ? "(unknown)" : cpu_model);
    summary += QString("  Cores : %1\n").arg(cpu_cores);
    summary += QString("  Load  : %1\n\n")
                   .arg(loadavg.isEmpty() ? "(unknown)" : loadavg);

    summary += "Memory (kB)\n";
    summary += QString("  Total     : %1\n").arg(mem_total_kb);
    summary += QString("  Available : %1\n\n").arg(mem_avail_kb);

    summary += "Network (bytes)\n";
    if (net.isEmpty())
    {
        summary += "  (no interfaces)\n";
    }
    else
    {
        summary += "  IFACE                 RX_BYTES           TX_BYTES\n";
        summary += "  --------------------------------------------------\n";
        for (const auto &v : net)
        {
            const QJsonObject n = v.toObject();
            const QString iface = n.value("iface").toString();
            const QString rx = QString::number(
                static_cast<qulonglong>(n.value("rx_bytes").toDouble()));
            const QString tx = QString::number(
                static_cast<qulonglong>(n.value("tx_bytes").toDouble()));
            summary += QString("  %1 %2 %3\n")
                           .arg(iface.leftJustified(20, ' '))
                           .arg(rx.rightJustified(15, ' '))
                           .arg(tx.rightJustified(15, ' '));
        }
    }

    summary += "\n=== JSON (Indented) ===\n";
    summary += QString::fromUtf8(doc.toJson(QJsonDocument::Indented));
    return summary;
}
} // namespace

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
                    const QString raw = QString::fromStdString(msg.data);
                    view_->setPlainText(to_pretty_view(raw));
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
    // 更适合“日志/JSON/表格”的等宽字体。
    QFont mono;
    mono.setStyleHint(QFont::Monospace);
    mono.setFamily("Monospace");
    text->setFont(mono);
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
