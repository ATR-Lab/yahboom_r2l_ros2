#include "yahboomcar_bringup/base.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
RobotBase::RobotBase() : Node("odometry_publisher"),
        last_vel_time_(this->now()),
        vel_dt_(0.0),
        x_pos_(0.0),
        y_pos_(0.0),
        heading_(0.0),
        linear_velocity_x_(0.0),
        linear_velocity_y_(0.0),
        angular_velocity_z_(0.0) {
    
    // Declare and get parameters
    this->declare_parameter("linear_scale_x", 1.0);
    this->declare_parameter("linear_scale_y", 1.0);
    this->declare_parameter("wheelbase", 0.25);
    this->declare_parameter("pub_odom_tf", false);
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_footprint_frame", "base_footprint");
    
    linear_scale_x = this->get_parameter("linear_scale_x").as_double();
    linear_scale_y = this->get_parameter("linear_scale_y").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    pub_odom_tf_ = this->get_parameter("pub_odom_tf").as_bool();
    odom_frame = this->get_parameter("odom_frame").as_string();
    base_footprint_frame = this->get_parameter("base_footprint_frame").as_string();
    
    // Create subscriber and publisher
    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/sub_vel", 50, std::bind(&RobotBase::velCallback, this, std::placeholders::_1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/pub_odom", 50);
    
    // Initialize transform broadcaster
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
//    ROS_ERROR_STREAM("linear_scale_x: "<<linear_scale_x);
//    ROS_ERROR_STREAM("linear_scale_y: "<<linear_scale_y);
//    ROS_ERROR_STREAM("odom_frame: "<<odom_frame);
//    ROS_ERROR_STREAM("base_footprint_frame: "<<base_footprint_frame);
}

//对于R2车型，vx为小车向前线速度，vy为舵机角度，vz为推算出的角速度。
void RobotBase::velCallback(const geometry_msgs::msg::Twist::SharedPtr twist) {
//    RCLCPP_INFO(this->get_logger(), "ODOM PUBLISH %.2f,%.2f,%.2f", twist->linear.x, twist->linear.y, twist->angular.z);
    rclcpp::Time current_time = this->now();
    linear_velocity_x_ = twist->linear.x * linear_scale_x;// scale = 1
    linear_velocity_y_ = twist->linear.y * linear_scale_y;
    //angular_velocity_z_ = twist->angular.z;   //We dont use this


    //calc time
    vel_dt_ = (current_time - last_vel_time_).seconds();
    last_vel_time_ = current_time;

    //compute odometry in a typical way given the velocities of the robot

    // wheelbase / R = tan(steer angle)
    double steer_angle = linear_velocity_y_;
    double MI_PI = 3.1416;
    double R = wheelbase_ / tan(steer_angle/180.0*MI_PI);
    double angular_velocity_z_ = linear_velocity_x_/R;

    //angular_velocity_z_ = twist.angular.z;   //We dont use this

    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_)) * vel_dt_; //m
    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;
    //calculate robot's heading in quaternion angle
    //ROS2 uses tf2 to calculate yaw in quaternion angle
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0.0, 0.0, heading_);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(tf_quat);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_footprint_frame;
    // robot's position in x,y and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    // robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;
    // linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    //odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.y = 0.0; // vy = 0.0
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    // angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;
    // RCLCPP_INFO(this->get_logger(), "ODOM PUBLISH");
    odom_publisher_->publish(odom);


    //publish odom tf
    if(pub_odom_tf_)
    {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = odom_frame;
        odom_tf.child_frame_id = base_footprint_frame;
        
        odom_tf.transform.translation.x = x_pos_;
        odom_tf.transform.translation.y = y_pos_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_quat;
        
        odom_broadcaster_->sendTransform(odom_tf);
        RCLCPP_INFO(this->get_logger(), "The transform has broadcasted!");
    }


}
