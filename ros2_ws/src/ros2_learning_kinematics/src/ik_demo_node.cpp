#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_kinematics/planar_2dof.hpp"
#include "geometry_msgs/msg/point.hpp"

class IkDemoNode : public rclcpp::Node
{
public:
    IkDemoNode() : Node("ik_demo_node")
    {
        // 初始化求解器 (L1=1.0m, L2=1.0m)
        solver_ = std::make_shared<ros2_learning_kinematics::Planar2DOF>(1.0, 1.0);

        // 创建一个定时器，每秒测试一个随机点
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&IkDemoNode::test_random_ik, this));
            
        RCLCPP_INFO(get_logger(), "IK Demo Node Started. Solving for Planar 2-DOF Arm...");
    }

private:
    void test_random_ik()
    {
        // 1. 生成一个“必定有解”的目标点
        // 我们先随机生成关节角，算出 FK，再把 FK 的结果作为 IK 的输入
        // 这样可以验证 IK 是否算对了
        
        double t1_target = (rand() % 628) / 100.0; // 0 ~ 6.28
        double t2_target = (rand() % 628) / 100.0;
        
        ros2_learning_kinematics::Planar2DOF::JointState js_target = {t1_target, t2_target};
        auto ee_target = solver_->forward(js_target);
        
        RCLCPP_INFO(get_logger(), "--------------------------------------------------");
        RCLCPP_INFO(get_logger(), "目标关节: [%.2f, %.2f]", t1_target, t2_target);
        RCLCPP_INFO(get_logger(), "目标末端: (%.2f, %.2f)", ee_target.x, ee_target.y);
        
        // 2. 调用 IK 求解
        auto solutions = solver_->inverse(ee_target);
        
        if (solutions.empty()) {
            RCLCPP_ERROR(get_logger(), "无解！(这不应该发生)");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "找到 %lu 组解:", solutions.size());
        for (size_t i = 0; i < solutions.size(); ++i) {
            auto& sol = solutions[i];
            
            // 验证解的准确性 (再算一次 FK)
            auto ee_check = solver_->forward(sol);
            double error = std::sqrt(std::pow(ee_check.x - ee_target.x, 2) + std::pow(ee_check.y - ee_target.y, 2));
            
            RCLCPP_INFO(get_logger(), "  解 %lu: [%.2f, %.2f] -> FK验证误差: %.6f", 
                i, sol.theta1, sol.theta2, error);
        }
    }

    std::shared_ptr<ros2_learning_kinematics::Planar2DOF> solver_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IkDemoNode>());
    rclcpp::shutdown();
    return 0;
}
