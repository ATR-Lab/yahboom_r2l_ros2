#ifndef YAHBOOM_BASE_H
#define YAHBOOM_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

class RobotBase : public rclcpp::Node {
public:
    RobotBase();

    void velCallback(const geometry_msgs::msg::Twist::SharedPtr twist);

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Time last_vel_time_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    string odom_frame;
    string base_footprint_frame;
    double linear_scale_x;
    double linear_scale_y;
    double wheelbase_;
    double vel_dt_;
    double x_pos_;
    double y_pos_;
    double heading_;
    double linear_velocity_x_;
    double linear_velocity_y_;
    double angular_velocity_z_;

    bool pub_odom_tf_;
};

#endif
