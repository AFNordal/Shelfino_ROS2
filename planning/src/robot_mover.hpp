#pragma once
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


using std::placeholders::_1;

class RobotMover : public rclcpp::Node
{
public:
    using FP = nav2_msgs::action::FollowPath;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FP>;

    RobotMover();

    void send_goal(FP::Goal goal_msg);

private:
    void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pathSubscription;
    rclcpp_action::Client<FP>::SharedPtr client_ptr_;
};