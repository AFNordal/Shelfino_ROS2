#pragma once


#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/graph.hpp"
#include <vector>
#include <limits>
#include <algorithm>

class TaskPlanner : public rclcpp::Node
{
public:
    TaskPlanner();

private:
    int num_nodes;
    std::vector<float> travel_distances;
    std::vector<int> profits;
    const int tmax = 12;

    void graphCallback(const interfaces::msg::Graph::SharedPtr msg);
    void calculatePath();

    rclcpp::Subscription<interfaces::msg::Graph>::SharedPtr subscription_;
};

