/**
 * @file   param_demo_node.cpp
 * @brief  参数进阶演示节点实现：声明、约束、回调校验与动态读取
 */

#include "ros2_learning_parameters_advanced/param_demo_node.hpp"

#include <limits>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace ros2_learning_parameters_advanced
{

// ─────────────────────────────────────────────────────────────────────────────
// 构造函数分为四个阶段，顺序不可颠倒：
//   阶段 1 — 声明全部参数（带描述符与范围约束）
//   阶段 2 — 校验 YAML/命令行注入的初值
//   阶段 3 — 注册参数修改前的校验回调
//   阶段 4 — 创建定时器
//
// 关键约束：add_on_set_parameters_callback 对 declare_parameter 阶段无效，
// 因此外部注入的初值必须在阶段 2 手动校验；而阶段 2 需要读取参数，
// 所以所有 declare_parameter 必须全部先于阶段 2 完成（阶段 1 不可分割）。
// ─────────────────────────────────────────────────────────────────────────────
ParamDemoNode::ParamDemoNode(const rclcpp::NodeOptions & options)
: Node("param_demo_node", options)
{
    // ── 阶段 1：声明全部参数 ──────────────────────────────────────────────────

    // print_rate_hz：整数，范围 [1, 100]
    // step = 1 表示只允许整数步长；from/to 均为闭区间
    {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = "打印频率（Hz），范围 [1, 100]";
        rcl_interfaces::msg::IntegerRange r;
        r.from_value = 1;
        r.to_value   = 100;
        r.step       = 1;
        d.integer_range.push_back(r);
        declare_parameter("print_rate_hz", 1, d);
    }

    // scale_factor：浮点，语义上是 (0.0, 10.0]（开区间下界）
    // 描述符下界设为 0.0（闭区间），由阶段 2 和回调共同保证 > 0 的精确语义。
    // 注意：不可用 double::min 近似下界——它会错误拒绝非规格化正数（如 1e-320）。
    {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = "消息缩放系数，范围 (0.0, 10.0]";
        rcl_interfaces::msg::FloatingPointRange r;
        r.from_value = 0.0;   // 描述符做粗筛（允许 >= 0），回调负责排除 == 0
        r.to_value   = 10.0;
        r.step       = 0.0;   // step = 0 表示无步长限制（允许任意精度浮点值）
        d.floating_point_range.push_back(r);
        declare_parameter("scale_factor", 1.0, d);
    }

    // node_label：字符串，自由格式，但不允许为空
    {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = "节点标签，任意非空字符串";
        declare_parameter("node_label", "default_label", d);
    }

    // verbose：布尔，开启后以 DEBUG 级别打印所有参数的详细值
    {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = "是否启用详细日志输出";
        declare_parameter("verbose", false, d);
    }

    // ── 阶段 2：校验外部注入的初值 ───────────────────────────────────────────
    // YAML 文件或命令行参数在 declare_parameter 时已写入节点状态，此处可读取。
    // 若初值非法则立即抛出异常，阻止节点进入 spinning 状态。
    {
        const double init_scale = get_parameter("scale_factor").as_double();
        if (init_scale <= 0.0) {
            RCLCPP_FATAL(get_logger(),
                "启动参数 scale_factor=%.6f 不合法（必须 > 0），节点拒绝启动", init_scale);
            throw std::invalid_argument("scale_factor must be > 0.0");
        }

        const auto init_label = get_parameter("node_label").as_string();
        if (init_label.empty()) {
            RCLCPP_FATAL(get_logger(), "启动参数 node_label 不能为空，节点拒绝启动");
            throw std::invalid_argument("node_label must not be empty");
        }
    }

    // ── 阶段 3：注册参数修改前的校验回调 ─────────────────────────────────────
    // 必须将返回的 SharedPtr 存入成员变量 param_cb_handle_；
    // 若仅赋给局部变量，handle 析构后回调立即被注销，校验失效。
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
            return on_parameters_set(params);
        });

    // ── 阶段 4：创建定时器 ────────────────────────────────────────────────────
    // 固定 1 s 周期打印当前参数值（演示目的，不实际用 print_rate_hz 控制周期）。
    // 若要动态调整周期，需在回调中重建定时器，超出本演示范围。
    timer_ = create_wall_timer(1s, std::bind(&ParamDemoNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
        "ParamDemoNode 已启动。\n"
        "  动态修改示例：ros2 param set /param_demo_node print_rate_hz 5\n"
        "  查看所有参数：ros2 param list /param_demo_node\n"
        "  查看描述符：  ros2 param describe /param_demo_node scale_factor");
}

// ─────────────────────────────────────────────────────────────────────────────
// 参数修改前校验回调
// 遇到第一个非法参数即拒绝整批修改（fail-fast 语义），
// 避免部分参数生效、部分被拒绝造成状态不一致。
// ─────────────────────────────────────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult
ParamDemoNode::on_parameters_set(const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;  // 默认通过，遇到非法值时覆写为 false

    for (const auto & p : params) {
        // node_label 不允许被设为空字符串（描述符无法表达"非空"约束，需手动校验）
        if (p.get_name() == "node_label" && p.as_string().empty()) {
            result.successful = false;
            result.reason = "node_label 不能为空字符串";
            RCLCPP_WARN(get_logger(), "参数校验失败：%s", result.reason.c_str());
            return result;
        }

        // scale_factor 必须严格大于 0（描述符 from_value = 0.0 允许零，回调在此排除）
        if (p.get_name() == "scale_factor" && p.as_double() <= 0.0) {
            result.successful = false;
            result.reason = "scale_factor 必须大于 0.0";
            RCLCPP_WARN(get_logger(), "参数校验失败：%s", result.reason.c_str());
            return result;
        }

        // 校验通过：以 DEBUG 级别记录本次修改（避免高频更新时日志开销过大）
        RCLCPP_DEBUG(get_logger(), "[回调] 参数 '%s' 更新为: %s",
            p.get_name().c_str(), p.value_to_string().c_str());
    }

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// 定时器回调：每秒打印一次当前参数快照
// ─────────────────────────────────────────────────────────────────────────────
void ParamDemoNode::on_timer()
{
    // 一次性读取全部参数，避免多次独立 get_parameter 调用
    const int    rate    = get_parameter("print_rate_hz").as_int();
    const double scale   = get_parameter("scale_factor").as_double();
    const auto   label   = get_parameter("node_label").as_string();
    const bool   verbose = get_parameter("verbose").as_bool();

    // INFO 级别始终输出参数摘要，方便观察动态修改效果
    RCLCPP_INFO(get_logger(),
        "[当前参数] label='%s'  rate=%d Hz  scale=%.3f  verbose=%s",
        label.c_str(), rate, scale, verbose ? "true" : "false");

    if (verbose) {
        // 演示 get_parameters 批量接口：一次 RCL 调用取多个参数
        for (const auto & p :
            get_parameters({"print_rate_hz", "scale_factor", "node_label", "verbose"}))
        {
            RCLCPP_DEBUG(get_logger(), "  [verbose] %s = %s",
                p.get_name().c_str(), p.value_to_string().c_str());
        }
    }
}

}  // namespace ros2_learning_parameters_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_parameters_advanced::ParamDemoNode)
