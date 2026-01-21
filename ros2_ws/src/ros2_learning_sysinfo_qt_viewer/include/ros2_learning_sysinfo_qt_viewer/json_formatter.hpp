// json_formatter.hpp
// JSON 解析和格式化工具

#pragma once

#include <QString>

namespace ros2_learning_sysinfo_qt_viewer
{

/**
 * @brief 将原始 JSON 字符串格式化为易读的仪表盘视图
 * 
 * 功能：
 * 1. 解析 JSON 数据
 * 2. 提取关键系统信息（主机名、CPU、内存、网络等）
 * 3. 格式化为表格式摘要
 * 4. 包含完整的 JSON 缩进输出
 * 
 * @param raw 原始 JSON 字符串
 * @return 格式化后的文本（包含摘要和完整 JSON）
 */
QString formatSystemInfo(const QString& raw);

} // namespace ros2_learning_sysinfo_qt_viewer
