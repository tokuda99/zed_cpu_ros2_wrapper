#include <zed_cpu_ros2_wrapper/zed_cpu_ros2_wrapper.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ZedCpuRos2Wrapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}