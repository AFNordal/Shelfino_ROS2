#pragma once


#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/graph.hpp"
#include "interfaces/msg/result.hpp"
#include <vector>
#include <limits>
#include <algorithm>


class TaskPlanner : public rclcpp::Node
{
public:
    TaskPlanner();
    void calculatePath(int currentNode, float currentDistance, int currentProfit,
                       std::vector<bool>& visited, int& maxProfit, std::vector<int>& pathVisited, std::vector<int>& bestPath, int& shortestPath);
     std::pair<int, std::vector<int>> findMaxProfit();

private:
    int num_nodes;
    std::vector<float> travel_distances;
    std::vector<int> profits;
    const int tmax = 30;

    void graphCallback(const interfaces::msg::Graph::SharedPtr msg);
    void calculatePath();
    float remainingDistance(int currentNode);
    void printPath(const std::vector<int>& pathVisited, int currentProfit);  // Helper function to print the path
    

    rclcpp::Subscription<interfaces::msg::Graph>::SharedPtr graph_subscription;
    rclcpp::Publisher<interfaces::msg::Result>::SharedPtr result_publisher;
};

