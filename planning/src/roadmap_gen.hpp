#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"

#include "planning/mapgeometry.hpp"
#include "planning/utils.hpp"
#include "planning/graphs.hpp"
#include "planning/dubins.hpp"

#include "hammersley/hammersley.hpp"

#define N_SAMPLES 10000
#define KNN_K 5
#define SHELFINO_TURNING_R 0.5 // REAL
// #define SHELFINO_TURNING_R 1.5
#define SHELFINO_VEL 0.8

using std::placeholders::_1;

typedef enum {
    PROBABILISTIC,
    COMBINATORIAL
} roadmapType;

class RoadmapGenerator : public rclcpp::Node
{
public:
    RoadmapGenerator();

    void dummy_border();
    void dummy_obstacles();
    void dummy_victims();
    void dummy_initPose();
    void dummy_shelfinoDescr();
    void dummy_gate();

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
    roadmapType strategy;

    void paths_from_roadmap();
    void minimal_clearance_graph(Graph &G, const std::vector<shared_ptr<Vertex>> &POIs);
    void generate_PRM(Graph &G);
    void smooth_bisect(const std::vector<std::shared_ptr<Vertex>> &path,
                       std::list<std::shared_ptr<Vertex>> &smooth,
                       size_t idx0,
                       size_t idx1,
                       const std::list<std::shared_ptr<Vertex>>::iterator &smooth_inserter);

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