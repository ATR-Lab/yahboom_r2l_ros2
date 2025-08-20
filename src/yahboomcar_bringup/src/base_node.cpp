#include <rclcpp/rclcpp.hpp>
#include "yahboomcar_bringup/base.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto robot_node = std::make_shared<RobotBase>();
    rclcpp::spin(robot_node);
    rclcpp::shutdown();
    return 0;
}
