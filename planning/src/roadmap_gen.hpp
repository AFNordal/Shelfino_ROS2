#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "planning/mapgeometry.hpp"
#include "planning/utils.hpp"
#include "planning/graphs.hpp"
#include "planning/dubins.hpp"

#include "hammersley/hammersley.hpp"
#include "interfaces/msg/graph.hpp"
#include "interfaces/msg/result.hpp"

#include <chrono>

#define N_SAMPLES 10000
#define KNN_K 5
#define SHELFINO_TURNING_R 0.5
#define SHELFINO_VEL 0.8

using std::chrono::high_resolution_clock;
typedef std::chrono::high_resolution_clock::time_point time_point;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;

using std::placeholders::_1;

typedef enum {
    PROBABILISTIC,
    COMBINATORIAL
} roadmapType;

class RoadmapGenerator : public rclcpp::Node
{
public:
    RoadmapGenerator();

private:
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borderSubscription;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscription;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victimsSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr shelfinoPoseSubscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shelfinoDescrSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gateSubscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tmax_subscription;
    rclcpp::Publisher<interfaces::msg::Graph>::SharedPtr graphPublisher;
    rclcpp::Subscription<interfaces::msg::Result>::SharedPtr TPResultSubscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pathPublisher;

    Map map;
    std::vector<std::vector<std::vector<Point_2>>> pathPointMatrix;
    std::vector<std::vector<double>> distMatrix;
    double tmax;
    bool planning_done = false;

    bool skip_shelfino;
    bool border_received, obstacles_received, victims_received, initPose_received, shelfinoDescr_received, gate_received, tmax_received = false;
    roadmapType strategy;

    void border_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void obstacles_callback(obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void victims_callback(obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void shelfinoPose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void shelfinoDescr_callback(std_msgs::msg::String::SharedPtr msg);
    void gate_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);
    void tmax_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void TPResult_callback(interfaces::msg::Result::SharedPtr msg);
    
    bool received_all()
    {
        return border_received &&
               obstacles_received &&
               victims_received &&
               initPose_received &&
               shelfinoDescr_received &&
               gate_received &&
               tmax_received;
    }
    void on_map_complete();
    void send_roadmap(void);
    void paths_from_roadmap();
    void minimal_clearance_graph(Graph &G, const std::vector<shared_ptr<Vertex>> &POIs);
    void generate_PRM(Graph &G);
    void smooth_bisect(const std::vector<std::shared_ptr<Vertex>> &path,
                       std::list<std::shared_ptr<Vertex>> &smooth,
                       size_t idx0,
                       size_t idx1,
                       const std::list<std::shared_ptr<Vertex>>::iterator &smooth_inserter);

    time_point t0;
    time_point t_paused;
    nanoseconds total_pause;
    bool timer_running = false;

    void start_timer() { assert(!timer_running); t0 = high_resolution_clock::now(); timer_running = true; }
    void pause_timer() { assert(timer_running); t_paused = high_resolution_clock::now(); timer_running = false; }
    void resume_timer() { assert(!timer_running); total_pause += high_resolution_clock::now() - t_paused; timer_running = true; }
    std::pair<long int, long int> read_timer() {
        auto _t = high_resolution_clock::now() - t0 - total_pause;
        auto _s = duration_cast<seconds>(_t).count();
        auto _ms = duration_cast<milliseconds>(_t).count() - 1000 * _s;
        return {_s, _ms};
    }
};
