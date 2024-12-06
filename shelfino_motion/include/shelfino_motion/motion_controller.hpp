#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include <cmath>
#include <iostream>

class MotionController : public rclcpp::Node
{
public:
    MotionController();
    void move_to_point(double x_goal, double y_goal);

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::Pose current_pose_;  // Current position of the robot

       // Timers
    rclcpp::TimerBase::SharedPtr simulated_odom_timer_;

    // Odometry Simulation
    void simulate_odometry();

    geometry_msgs::msg::Twist last_cmd_;
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
};



#endif  // SHELFINO_MOTION_MOTION_CONTROLLER_HPP_
