#include "shelfino_motion/motion_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

MotionController::MotionController() : Node("motion_controller")
{
    // Publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/shelfino1/cmd_vel", 10);

    // Subscriber for odometry feedback from Gazebo
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/shelfino1/odom", 10, std::bind(&MotionController::odom_callback, this, std::placeholders::_1));
}

void MotionController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_ = msg->pose.pose;

    tf2::Quaternion q(
        current_pose_.orientation.x,
        current_pose_.orientation.y,
        current_pose_.orientation.z,
        current_pose_.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(), "Odometry Callback: x=%.2f, y=%.2f, yaw=%.2f",
                current_pose_.position.x, current_pose_.position.y, yaw);
}

void MotionController::move_to_point(double x_goal, double y_goal)
{
    RCLCPP_INFO(this->get_logger(), "Moving to point: x=%.2f, y=%.2f", x_goal, y_goal);

    geometry_msgs::msg::Twist cmd_msg;
    int alignment_attempts = 0; // Count alignment attempts

    while (rclcpp::ok()) {
        // Spin to process odometry callbacks
        rclcpp::spin_some(this->get_node_base_interface());

        // Calculate the difference between the current position and the goal
        double dx = x_goal - current_pose_.position.x;
        double dy = y_goal - current_pose_.position.y;

        // Calculate distance to the goal
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);
        if (distance_to_goal < 0.05) {  // Threshold to consider the goal reached
            RCLCPP_INFO(this->get_logger(), "Reached goal: x=%.2f, y=%.2f", x_goal, y_goal);
            break;
        }

        // Calculate angle to the goal
        double angle_to_goal = std::atan2(dy, dx);

        // Get the robot's current orientation (yaw) from quaternion
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Calculate angular error
        double angular_error = angle_to_goal - yaw;

        // Normalize angular error to [-pi, pi]
        while (angular_error > M_PI) angular_error -= 2 * M_PI;
        while (angular_error < -M_PI) angular_error += 2 * M_PI;

        RCLCPP_INFO(this->get_logger(),
        "Goal position: x= %.2f, y=%.2f", x_goal, y_goal);

        RCLCPP_INFO(this->get_logger(),
                    "Current: x=%.2f, y=%.2f, Yaw=%.2f, Distance to goal=%.2f, Angle=%.2f, Angular Error=%.2f",
                    current_pose_.position.x, current_pose_.position.y, yaw, distance_to_goal, angle_to_goal, angular_error);

        // Control logic for movement
        if (std::abs(angular_error) > 0.4) {
            // Prioritize rotation if angular error is significant
            cmd_msg.angular.z = 0.3 * angular_error;
            cmd_msg.linear.x = 0.0;
            alignment_attempts++;
/*
            if (alignment_attempts > 50) { // Threshold for giving up alignment
                RCLCPP_WARN(this->get_logger(), "Alignment timeout. Switching to forward motion.");
                cmd_msg.angular.z = 0.0;
                cmd_msg.linear.x = 0.5; // Slow forward motion
            }
            */
        } else {
            // Reset counter and transition to forward motion
            alignment_attempts = 0;
            cmd_msg.angular.z = 0.0;
            cmd_msg.linear.x = std::min(0.5, 0.5 * distance_to_goal); // Scale speed with distance
        }

        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_msg);

        // Sleep briefly to allow the robot to execute the command
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the robot after reaching the goal
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_msg);

    RCLCPP_INFO(this->get_logger(), "Stopped at goal: x=%.2f, y=%.2f", x_goal, y_goal);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto motion_controller = std::make_shared<MotionController>();

    std::cout << "Enter points in the format: x y" << std::endl;
    std::cout << "Type 'exit' to quit." << std::endl;

    while (rclcpp::ok()) {
        double x, y;
        std::cout << "Enter coordinates: ";
        std::cin >> x >> y;

        if (std::cin.fail()) {
            std::cerr << "Invalid input. Please enter valid numeric values for x and y." << std::endl;
            std::cin.clear();
            std::cin.ignore(1000, '\n');
            continue;
        }

        motion_controller->move_to_point(x, y);
    }

    rclcpp::shutdown();
    return 0;
}
