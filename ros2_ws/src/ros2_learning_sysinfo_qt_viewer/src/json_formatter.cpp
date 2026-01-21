// json_formatter.cpp
// JSON 解析和格式化工具实现

#include "ros2_learning_sysinfo_qt_viewer/json_formatter.hpp"

#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace ros2_learning_sysinfo_qt_viewer
{

QString formatSystemInfo(const QString& raw)
{
    // 目标：
    // 1) 如果是 JSON：用 Qt 解析后缩进打印，便于肉眼阅读；
    // 2) 同时输出"摘要区"，把关键信息提取出来（更像仪表盘）。
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
        for (const auto& v : net)
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

} // namespace ros2_learning_sysinfo_qt_viewer
