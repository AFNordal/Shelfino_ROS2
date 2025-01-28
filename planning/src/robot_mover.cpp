#include "robot_mover.hpp"

RobotMover::RobotMover() : Node("robot_mover")
{
    rclcpp::QoS TL_qos(rclcpp::KeepLast(1));
    TL_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    pathSubscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/planned_path", TL_qos, std::bind(&RobotMover::path_callback, this, _1));

    this->client_ptr_ = rclcpp_action::create_client<FP>(
        this,
        "/shelfino/follow_path");
}

void RobotMover::path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (path_received)
        return;
    path_received = true;
    RCLCPP_INFO(this->get_logger(), "Received path");
    auto goal_msg = FP::Goal();
    goal_msg.controller_id = "FollowPath";
    goal_msg.goal_checker_id = "goal_checker";
    goal_msg.path.header.frame_id = "map";
    goal_msg.path.header.stamp = this->now();
    for (auto &p : msg->poses)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose = p;
        ps.header.frame_id = "map";
        ps.header.stamp = this->now();
        goal_msg.path.poses.push_back(ps);
    }
    send_goal(goal_msg);
}

void RobotMover::send_goal(FP::Goal goal_msg)
{
    if (!this->client_ptr_->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FP>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    };

    send_goal_options.feedback_callback = [this](
                                              GoalHandle::SharedPtr,
                                              const std::shared_ptr<const FP::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Moving at speed %f", feedback->speed);
    };

    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        //   for (auto number : result.result->sequence) {
        //     ss << number << " ";
        //   }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    };
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMover>());
    rclcpp::shutdown();

    return 0;
}