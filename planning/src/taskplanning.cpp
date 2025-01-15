
#include "taskplanning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/graph.hpp"  // Include the custom message header
#include <vector>
#include "taskplanning.hpp"

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    subscription_ = this->create_subscription<interfaces::msg::Graph>(
        "/graph_topic", 10,
        std::bind(&TaskPlanner::graphCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Task Planner node started.");
}

void TaskPlanner::graphCallback(const interfaces::msg::Graph::SharedPtr msg)
{
    num_nodes = msg->num_nodes;
    travel_distances = msg->travel_distance;
    profits = msg->profits;

    RCLCPP_INFO(this->get_logger(), "Received Graph message:");
    RCLCPP_INFO(this->get_logger(), "Number of nodes: %d", msg->num_nodes);
    RCLCPP_INFO(this->get_logger(), "Travel distances:");
    for (const auto &distance : msg->travel_distance)
    {
        RCLCPP_INFO(this->get_logger(), "  - %f", distance);
    }
    RCLCPP_INFO(this->get_logger(), "Profits:");
    for (const auto &profit : msg->profits)
    {
        RCLCPP_INFO(this->get_logger(), "  - %d", profit);
    }

    calculatePath();
}

void TaskPlanner::calculatePath() { 
    // Ensure the travel distances matrix is properly sized
    if (travel_distances.size() != num_nodes * num_nodes) {
        std::cerr << "Error: Travel distances matrix size is incorrect.\n";
        return;
    }

    // DP table to store the maximum profit at each node within tmax time
    std::vector<std::vector<int>> dp(num_nodes, std::vector<int>(tmax, -1)); // tmax time slots
    // Path tracking table to reconstruct the best path
    std::vector<std::vector<int>> prev(num_nodes, std::vector<int>(tmax, -1));

    // Initialize the starting node
    dp[0][0] = profits[0];

    // Fill the DP table
    for (int time = 0; time < tmax; ++time) {
        for (int u = 0; u < num_nodes; ++u) {
            if (dp[u][time] == -1) continue; // Skip unreachable states
            for (int v = 0; v < num_nodes; ++v) {
                if (u == v) continue; // Skip self-loops
                int travel_time = static_cast<int>(travel_distances[u * num_nodes + v]);
                if (time + travel_time < tmax) { // Total time must be strictly less than tmax
                    int new_profit = dp[u][time] + profits[v];
                    if (new_profit > dp[v][time + travel_time]) {
                        dp[v][time + travel_time] = new_profit;
                        prev[v][time + travel_time] = u; // Track the path
                    }
                }
            }
        }
    }

    // Find the maximum profit at the last node within tmax - 1 time
    int max_profit = -1;
    int best_time = -1;
    for (int time = 0; time < tmax; ++time) {
        if (dp[num_nodes - 1][time] > max_profit) {
            max_profit = dp[num_nodes - 1][time];
            best_time = time;
        }
    }

    // Output the result
    if (max_profit == -1) {
        std::cout << "No valid path found within time constraints.\n";
        return;
    }

    std::cout << "Maximum profit: " << max_profit << "\n";

    // Reconstruct the path
    std::vector<int> path;
    int current_node = num_nodes - 1;
    int current_time = best_time;

    while (current_node != -1) {
        path.push_back(current_node);
        current_node = prev[current_node][current_time];
        if (current_node != -1) {
            current_time -= static_cast<int>(travel_distances[current_node * num_nodes + path.back()]);
        }
    }

    std::reverse(path.begin(), path.end());

    std::cout << "Best path: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << "\n";
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



