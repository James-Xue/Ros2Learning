#include "ros2_learning_cpp/talker.hpp"

using namespace std::chrono_literals;

namespace ros2_learning_cpp
{
    // 组件构造函数，改为继承 LifecycleNode
    TalkerNode::TalkerNode(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("talker_lifecycle", options)
    {
        // 声明参数
        this->declare_parameter("message_text", "Hello Lifecycle World");
        
        // 构造时不再直接创建发布者，而是等到 Configure 阶段
        RCLCPP_INFO(get_logger(), "on_create: Unconfigured state");
    }

    // 1. Configure: Unconfigured -> Inactive
    TalkerNode::CallbackReturn
    TalkerNode::on_configure(const rclcpp_lifecycle::State &)
    {
        // 创建发布者，但此时它还不会真正发布数据
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        
        // 创建定时器，但先不要启动它 (我们用一个小技巧，在 Activate 时再让它生效，或者在这里创建但在回调里检查状态)
        // 更好的做法是：在 activate 时创建 timer，在 deactivate 时销毁 timer。
        // 为了演示方便，我们这里先创建，然后在回调里判断。
        timer_ = this->create_wall_timer(500ms, std::bind(&TalkerNode::on_timer, this));
        
        // 实际上，rclcpp_lifecycle 的 Timer 并没有直接的 enable/disable 接口像 Publisher 那样自动管理。
        // 所以严格来说，应该在 on_activate 里 create_wall_timer，在 on_deactivate 里 timer_->cancel() 或 reset。
        // 这里为了简单，我们先创建，但由于 Publisher 是 LifecyclePublisher，它在 Inactive 状态下调用 publish 是无效的，所以也安全。

        RCLCPP_INFO(get_logger(), "on_configure: Configured! Now Inactive.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // 2. Activate: Inactive -> Active
    TalkerNode::CallbackReturn
    TalkerNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        // 必须调用父类的 on_activate 来激活 LifecyclePublisher
        LifecycleNode::on_activate(state);
        
        // 可以在这里重置计数器
        // count_ = 0;

        RCLCPP_INFO(get_logger(), "on_activate: Activated! Now Publishing.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // 3. Deactivate: Active -> Inactive
    TalkerNode::CallbackReturn
    TalkerNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        // 必须调用父类的 on_deactivate 来停用 LifecyclePublisher
        LifecycleNode::on_deactivate(state);

        RCLCPP_INFO(get_logger(), "on_deactivate: Deactivated! Stopped Publishing.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // 4. Cleanup: Inactive -> Unconfigured
    TalkerNode::CallbackReturn
    TalkerNode::on_cleanup(const rclcpp_lifecycle::State &)
    {
        // 销毁发布者和定时器
        publisher_.reset();
        timer_.reset();

        RCLCPP_INFO(get_logger(), "on_cleanup: Cleaned up! Back to Unconfigured.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // 5. Shutdown: Any -> Finalized
    TalkerNode::CallbackReturn
    TalkerNode::on_shutdown(const rclcpp_lifecycle::State &)
    {
        publisher_.reset();
        timer_.reset();
        RCLCPP_INFO(get_logger(), "on_shutdown: Shutting down.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void TalkerNode::on_timer()
    {
        // 只有在 Active 状态下，LifecyclePublisher 才会真正把数据发出去
        if (publisher_->is_activated()) {
            std::string message_text = this->get_parameter("message_text").as_string();
            std_msgs::msg::String msg;
            msg.data = message_text + ", count=" + std::to_string(count_++);
            RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
            publisher_->publish(msg);
        }
    }
}

// 注册组件
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_learning_cpp::TalkerNode)
