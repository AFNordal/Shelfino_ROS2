#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"

#include <CGAL/Point_set_2.h>

#include "planning/mapgeometry.hpp"
#include "planning/utils.hpp"
#include "planning/graphs.hpp"

#include "hammersley/hammersley.hpp"

typedef CGAL::Point_set_2<K> Point_set_2;

namespace plt = matplotlibcpp;
using std::placeholders::_1;

class RoadmapGenerator : public rclcpp::Node
{
public:
    RoadmapGenerator();

private:
    void on_map_complete();

    void border_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void obstacles_callback(obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void victims_callback(obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void initPose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void shelfinoDescr_callback(std_msgs::msg::String::SharedPtr msg);
    void gate_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borderSubscription;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscription;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victimsSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initPoseSubscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shelfinoDescrSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gateSubscription;

    bool border_received, obstacles_received, victims_received, initPose_received, shelfinoDescr_received, gate_received = false;

    Map map;
    bool received_all()
    {
        return border_received && 
               obstacles_received && 
               victims_received && 
               initPose_received &&
               shelfinoDescr_received &&
               gate_received;
    }
};