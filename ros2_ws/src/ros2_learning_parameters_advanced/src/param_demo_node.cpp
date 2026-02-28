#include "ros2_learning_parameters_advanced/param_demo_node.hpp"

#include <limits>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace ros2_learning_parameters_advanced
{

ParamDemoNode::ParamDemoNode(const rclcpp::NodeOptions & options)
: Node("param_demo_node", options)
{
    // ----------------------------------------------------------------
    // 1. declare_parameter — 带描述符（范围约束 + 描述文字）
    // ----------------------------------------------------------------

    // 整数参数：发布频率（Hz），限定范围 [1, 100]
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "打印频率（Hz），范围 [1, 100]";
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = 1;
        range.to_value   = 100;
        range.step       = 1;
        desc.integer_range.push_back(range);
        this->declare_parameter("print_rate_hz", 1, desc);
    }

    // 浮点参数：缩放系数，限定范围 (0.0, 10.0]
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "消息缩放系数，范围 (0.0, 10.0]";
        // 注意：ROS 2 描述符范围是闭区间，无法直接表达开区间下界 > 0
        // 因此将 from_value 设为最小正精度；额外校验在回调中强制执行
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = std::numeric_limits<double>::min();  // >0 的最小正数
        range.to_value   = 10.0;
        range.step       = 0.0;  // 0 表示无步长限制
        desc.floating_point_range.push_back(range);
        this->declare_parameter("scale_factor", 1.0, desc);
    }

    // 在所有参数声明完成后，检验来自 YAML/命令行的初值是否合法
    // （add_on_set_parameters_callback 不会覆盖 declare_parameter 阶段注入的值）
    {
        const double init_scale = this->get_parameter("scale_factor").as_double();
        if (init_scale <= 0.0) {
            RCLCPP_FATAL(get_logger(),
                "启动参数 scale_factor=%.6f 不合法（必须 > 0），节点拒绝启动", init_scale);
            throw std::invalid_argument("scale_factor must be > 0.0");
        }
        const std::string init_label = this->get_parameter("node_label").as_string();
        if (init_label.empty()) {
            RCLCPP_FATAL(get_logger(), "启动参数 node_label 不能为空，节点拒绝启动");
            throw std::invalid_argument("node_label must not be empty");
        }
    }

    // 字符串参数：节点标签（自由格式，不校验）
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "节点标签，任意字符串";
        this->declare_parameter("node_label", "default_label", desc);
    }

    // 布尔参数：是否启用详细输出
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "是否启用详细日志输出";
        this->declare_parameter("verbose", false, desc);
    }

    // ----------------------------------------------------------------
    // 2. add_on_set_parameters_callback — 参数修改前的校验回调
    //    注意：必须将返回的 handle 存入成员变量，否则回调立即失效
    // ----------------------------------------------------------------
    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
            return this->on_parameters_set(params);
        });

    // ----------------------------------------------------------------
    // 3. 定时器：周期由 print_rate_hz 参数决定
    //    使用固定 1s 定时器，内部读取最新参数（简化演示）
    // ----------------------------------------------------------------
    timer_ = this->create_wall_timer(1s, std::bind(&ParamDemoNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
        "ParamDemoNode 已启动。\n"
        "  动态修改示例：ros2 param set /param_demo_node print_rate_hz 5\n"
        "  查看所有参数：ros2 param list /param_demo_node\n"
        "  查看描述符：  ros2 param describe /param_demo_node scale_factor");
}

// ----------------------------------------------------------------
// 参数校验回调
// ----------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult
ParamDemoNode::on_parameters_set(const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
        // 额外校验：node_label 不允许为空字符串
        if (p.get_name() == "node_label") {
            if (p.as_string().empty()) {
                result.successful = false;
                result.reason = "node_label 不能为空字符串";
                RCLCPP_WARN(get_logger(), "参数校验失败：%s", result.reason.c_str());
                return result;
            }
        }

        // 额外校验：scale_factor 必须 > 0（描述符中 from_value=0 是闭区间，需手动排除 0）
        if (p.get_name() == "scale_factor") {
            if (p.as_double() <= 0.0) {
                result.successful = false;
                result.reason = "scale_factor 必须大于 0.0";
                RCLCPP_WARN(get_logger(), "参数校验失败：%s", result.reason.c_str());
                return result;
            }
        }

        RCLCPP_INFO(get_logger(), "[回调] 参数 '%s' 更新为: %s",
            p.get_name().c_str(), p.value_to_string().c_str());
    }

    return result;
}

// ----------------------------------------------------------------
// 定时打印当前参数值
// ----------------------------------------------------------------
void ParamDemoNode::on_timer()
{
    const int    rate    = this->get_parameter("print_rate_hz").as_int();
    const double scale   = this->get_parameter("scale_factor").as_double();
    const auto   label   = this->get_parameter("node_label").as_string();
    const bool   verbose = this->get_parameter("verbose").as_bool();

    RCLCPP_INFO(get_logger(),
        "[当前参数] label='%s'  rate=%d Hz  scale=%.3f  verbose=%s",
        label.c_str(), rate, scale, verbose ? "true" : "false");

    if (verbose) {
        // 演示用 get_parameters 批量获取
        auto all = this->get_parameters({"print_rate_hz", "scale_factor", "node_label", "verbose"});
        for (const auto & p : all) {
            RCLCPP_DEBUG(get_logger(), "  [verbose] %s = %s",
                p.get_name().c_str(), p.value_to_string().c_str());
        }
    }
}

}  // namespace ros2_learning_parameters_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_parameters_advanced::ParamDemoNode)
