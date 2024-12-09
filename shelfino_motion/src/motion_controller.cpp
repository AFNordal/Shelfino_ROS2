#include "shelfino_motion/motion_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <utility>
#include <iostream>



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

    RCLCPP_INFO(this->get_logger(), "Odometry Callback: x=%.2f, y=%.2f, yaw=%.2f",current_pose_.position.x, current_pose_.position.y, yaw);
}

void MotionController::move_to_point(double x_goal, double y_goal, bool has_next_point, double next_x = 0.0, double next_y = 0.0)
{
    geometry_msgs::msg::Twist cmd_msg;
    double linear_speed = 0.5;  
    double angular_gain = 1.0; 
    double angular_tolerance = 0.1; 
    while (rclcpp::ok()) {

        
        rclcpp::spin_some(this->get_node_base_interface());

        
        double dx = x_goal - current_pose_.position.x;
        double dy = y_goal - current_pose_.position.y;

        
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);

        // Check if goal is reached
        if (!has_next_point && distance_to_goal < 0.2) {
            RCLCPP_INFO(this->get_logger(), "Reached endpoint: x=%.2f, y=%.2f", x_goal, y_goal);
            break;
        }

      
        double angle_to_goal = std::atan2(dy, dx);

        // If there's a next point, adjust angle to consider it
        if (has_next_point) {
            double next_dx = next_x - current_pose_.position.x;
            double next_dy = next_y - current_pose_.position.y;
            angle_to_goal = std::atan2(next_dy, next_dx);
        }

        
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

     
        double angular_error = angle_to_goal - yaw;
        while (angular_error > M_PI) angular_error -= 2 * M_PI;
        while (angular_error < -M_PI) angular_error += 2 * M_PI;

        // Control angular velocity to reduce error
        if (std::abs(angular_error) > angular_tolerance) {
            cmd_msg.angular.z = angular_gain * angular_error;
            cmd_msg.linear.x = 0.0; 
        } else {
            
            cmd_msg.angular.z = angular_gain * angular_error;
            cmd_msg.linear.x = linear_speed;
        }

       
        cmd_vel_pub_->publish(cmd_msg);

      
        if (has_next_point && distance_to_goal < 0.2) {
            RCLCPP_INFO(this->get_logger(), "Approaching point: x=%.2f, y=%.2f, proceeding to next.", x_goal, y_goal);
            return;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    // Stop robot only at the final point
    if (!has_next_point) {
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_msg);
    }
}



//main for predefined vector
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto motion_controller = std::make_shared<MotionController>();


/*
    // Define a predefined swing path with close points
    std::vector<std::pair<double, double>> points = {
        {0.0, 0.0},
        {0.1, 0.05},
        {0.2, 0.1},
        {0.3, 0.15},
        {0.4, 0.2},
        {0.5, 0.25},
        {0.6, 0.3},
        {0.7, 0.35},
        {0.8, 0.4},
        {0.9, 0.45},
        {1.0, 0.5}
    };
*/
    std::vector<std::pair<double, double>> points = {
    {1.0, 2.0}, {1.032, 2.097}, {1.065, 2.194}, {1.097, 2.290}, {1.129, 2.387},
    {1.161, 2.484}, {1.194, 2.581}, {1.226, 2.677}, {1.258, 2.774}, {1.290, 2.871},
    {1.323, 2.968}, {1.355, 3.065}, {1.387, 3.161}, {1.419, 3.258}, {1.452, 3.355},
    {1.484, 3.452}, {1.516, 3.548}, {1.548, 3.645}, {1.581, 3.742}, {1.613, 3.839},
    {1.645, 3.935}, {1.677, 4.032}, {1.710, 4.129}, {1.742, 4.226}, {1.774, 4.323},
    {1.807, 4.419}, {1.839, 4.516}, {1.871, 4.613}, {1.903, 4.710}, {1.936, 4.806},
    {1.968, 4.903}, {2.0, 5.0}, {2.056, 4.917}, {2.111, 4.833}, {2.167, 4.750},
    {2.222, 4.667}, {2.278, 4.583}, {2.333, 4.500}, {2.389, 4.417}, {2.444, 4.333},
    {2.500, 4.250}, {2.556, 4.167}, {2.611, 4.083}, {2.667, 4.0}, {2.722, 3.917},
    {2.778, 3.833}, {2.833, 3.750}, {2.889, 3.667}, {2.944, 3.583}, {3.0, 3.5},
    {3.056, 3.417}, {3.111, 3.333}, {3.167, 3.250}, {3.222, 3.167}, {3.278, 3.083},
    {3.333, 3.0}, {3.389, 2.917}, {3.444, 2.833}, {3.5, 2.75}, {3.556, 2.667},
    {3.611, 2.583}, {3.667, 2.5}, {3.722, 2.417}, {3.778, 2.333}, {3.833, 2.25},
    {3.889, 2.167}, {3.944, 2.083}, {4.0, 2.0}, {4.046, 2.091}, {4.091, 2.182},
    {4.136, 2.273}, {4.182, 2.364}, {4.227, 2.455}, {4.273, 2.545}, {4.318, 2.636},
    {4.364, 2.727}, {4.409, 2.818}, {4.455, 2.909}, {4.5, 3.0}, {4.545, 3.091},
    {4.591, 3.182}, {4.636, 3.273}, {4.682, 3.364}, {4.727, 3.455}, {4.773, 3.545},
    {4.818, 3.636}, {4.864, 3.727}, {4.909, 3.818}, {4.955, 3.909}, {5.0, 4.0},
    {5.056, 4.083}, {5.111, 4.167}, {5.167, 4.25}, {5.222, 4.333}, {5.278, 4.417},
    {5.333, 4.5}, {5.389, 4.583}, {5.444, 4.667}, {5.5, 4.75}, {5.556, 4.833},
    {5.611, 4.917}, {5.667, 5.0}, {5.722, 5.083}, {5.778, 5.167}, {5.833, 5.25},
    {5.889, 5.333}, {5.944, 5.417}, {6.0, 5.5}, {6.056, 5.583}, {6.111, 5.667},
    {6.167, 5.75}, {6.222, 5.833}, {6.278, 5.917}, {6.333, 6.0}, {6.389, 6.083},
    {6.444, 6.167}, {6.5, 6.25}, {6.556, 6.333}, {6.611, 6.417}, {6.667, 6.5},
    {6.722, 6.583}, {6.778, 6.667}, {6.833, 6.75}, {6.889, 6.833}, {6.944, 6.917},
    {7.0, 7.0}, {6.929, 6.929}, {6.857, 6.857}, {6.786, 6.786}, {6.714, 6.714},
    {6.643, 6.643}, {6.571, 6.571}, {6.5, 6.5}, {6.429, 6.429}, {6.357, 6.357},
    {6.286, 6.286}, {6.214, 6.214}, {6.143, 6.143}, {6.071, 6.071}, {6.0, 6.0},
    {5.929, 5.929}, {5.857, 5.857}, {5.786, 5.786}, {5.714, 5.714}, {5.643, 5.643},
    {5.571, 5.571}, {5.5, 5.5}, {5.429, 5.429}, {5.357, 5.357}, {5.286, 5.286},
    {5.214, 5.214}, {5.143, 5.143}, {5.071, 5.071}, {5.0, 5.0}, {4.929, 4.929},
    {4.857, 4.857}, {4.786, 4.786}, {4.714, 4.714}, {4.643, 4.643}, {4.571, 4.571},
    {4.5, 4.5}, {4.429, 4.429}, {4.357, 4.357}, {4.286, 4.286}, {4.214, 4.214},
    {4.143, 4.143}, {4.071, 4.071}, {4.0, 4.0}, {3.929, 3.929}, {3.857, 3.857},
    {3.786, 3.786}, {3.714, 3.714}, {3.643, 3.643}, {3.571, 3.571}, {3.5, 3.5},
    {3.429, 3.429}, {3.357, 3.357}, {3.286, 3.286}, {3.214, 3.214}, {3.143, 3.143},
    {3.071, 3.071}, {3.0, 3.0}, {2.929, 2.929}, {2.857, 2.857}, {2.786, 2.786},
    {2.714, 2.714}, {2.643, 2.643}, {2.571, 2.571}, {2.5, 2.5}, {2.429, 2.429},
    {2.357, 2.357}, {2.286, 2.286}, {2.214, 2.214}, {2.143, 2.143}, {2.071, 2.071},
    {2.0, 2.0}, {1.929, 1.929}, {1.857, 1.857}, {1.786, 1.786}, {1.714, 1.714},
    {1.643, 1.643}, {1.571, 1.571}, {1.5, 1.5}, {1.429, 1.429}, {1.357, 1.357},
    {1.286, 1.286}, {1.214, 1.214}, {1.143, 1.143}, {1.071, 1.071}, {1.0, 1.0},
    {0.929, 0.929}, {0.857, 0.857}, {0.786, 0.786}, {0.714, 0.714}, {0.643, 0.643},
    {0.571, 0.571}, {0.5, 0.5}, {0.429, 0.429}, {0.357, 0.357}, {0.286, 0.286},
    {0.214, 0.214}, {0.143, 0.143}, {0.071, 0.071}, {0.0, 0.0}, {0, 0}
};

    std::cout << "Moving through the predefined swing path:" << std::endl;
    for (const auto &point : points) {
        std::cout << "(" << point.first << ", " << point.second << ")" << std::endl;
    }

   
    for (size_t i = 0; i < points.size(); ++i) {
        double x_goal = points[i].first;
        double y_goal = points[i].second;

        bool has_next_point = (i + 1 < points.size());
        double next_x = has_next_point ? points[i + 1].first : 0.0;
        double next_y = has_next_point ? points[i + 1].second : 0.0;

        motion_controller->move_to_point(x_goal, y_goal, has_next_point, next_x, next_y);
    }

    rclcpp::shutdown();
    return 0;
}



/*
//Main for defined vector from terminal
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto motion_controller = std::make_shared<MotionController>();

    std::vector<std::pair<double, double>> points;
    std::cout << "Enter points in the format: x y (Type 'done' to finish entering points)" << std::endl;

    while (true) {
        std::string input;
        std::cout << "Enter coordinates or 'done': ";
        std::getline(std::cin, input);

        if (input == "done") {
            break;
        }

        std::istringstream iss(input);
        double x, y;
        if (!(iss >> x >> y)) {
            std::cerr << "Invalid input. Please enter valid numeric values for x and y." << std::endl;
            continue;
        }

        points.emplace_back(x, y);
    }

    if (points.empty()) {
        std::cout << "No points entered. Exiting." << std::endl;
        rclcpp::shutdown();
        return 0;
    }

    std::cout << "Moving through the following points:" << std::endl;
    for (const auto &point : points) {
        std::cout << "(" << point.first << ", " << point.second << ")" << std::endl;
    }

    for (const auto &point : points) {
        motion_controller->move_to_point(point.first, point.second);
    }

    rclcpp::shutdown();
    return 0;
}
*/