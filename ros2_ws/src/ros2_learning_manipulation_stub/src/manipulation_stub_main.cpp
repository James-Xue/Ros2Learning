#include "manipulation_stub.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<ManipulationStub>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "[manipulation_stub] FATAL: exception: %s\n", e.what());
        rclcpp::shutdown();
        return 2;
    }
}
