#include "taskplanning.hpp"

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    rclcpp::QoS TL_qos(rclcpp::KeepLast(1));
    TL_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    graph_subscription = this->create_subscription<interfaces::msg::Graph>(
        "/graph_topic", TL_qos,
        std::bind(&TaskPlanner::graphCallback, this, std::placeholders::_1));
    tmax_subscription = this->create_subscription<std_msgs::msg::Int32>(
        "/victims_timeout", TL_qos,
        std::bind(&TaskPlanner::tmaxCallback, this, std::placeholders::_1));
    result_publisher = this->create_publisher<interfaces::msg::Result>("/resultTP_topic", TL_qos);
    RCLCPP_INFO(this->get_logger(), "Task Planner node started.");
}

void TaskPlanner::tmaxCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (tmax_received)
        return;
    tmax_received = true;
    tmax = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received tmax %d", tmax);
    if (graph_received && tmax_received)
        onAllReceived();
}

float TaskPlanner::remainingDistance(int currentNode)
{
    return travel_distances[currentNode * num_nodes + (num_nodes - 1)];
}

// recursive function to explore all possible paths
void TaskPlanner::calculatePath(int currentNode, float currentDistance, int currentProfit,
                                std::vector<bool> &visited, int &maxProfit, std::vector<int> &pathVisited, std::vector<int> &bestPath, int &shortestPath)
{
    printPath(pathVisited, currentProfit);
    // Before exploring any path, check if it's still possible to reach the last node within tmax
    float remaining = remainingDistance(currentNode);
    if (currentDistance + remaining > tmax)
    {
        return; // If not possible, quit exploring this path
    }

    if (currentNode == num_nodes - 1)
    {
        // Update the maximum profit and shortest path if it is better than what we have previously found
        if ((currentProfit > maxProfit && currentProfit > 0) || (currentProfit == maxProfit && currentDistance < shortestPath))
        {
            maxProfit = currentProfit;
            shortestPath = currentDistance;
            bestPath.clear();
            for (size_t i = 0; i < pathVisited.size(); i++)
            {
                bestPath.push_back(static_cast<int>(pathVisited[i]));
            }
            std::cout << "length: " << shortestPath << std::endl;
            printPath(bestPath, currentProfit); // Print the path and profit when maxProfit is updated
        }
        return;
    }
    for (int nextNode = 1; nextNode < num_nodes; ++nextNode)
    {
        if (!(nextNode == currentNode) && !(visited[nextNode]))
        {
            float distance = travel_distances[currentNode * num_nodes + nextNode];
            int profit = profits[nextNode];

            if (distance > 0)
            {
                visited[nextNode] = true;
                pathVisited.push_back(nextNode);
                calculatePath(nextNode, currentDistance + distance, currentProfit + profit,
                              visited, maxProfit, pathVisited, bestPath, shortestPath);
            }
            visited[nextNode] = false;
            pathVisited.pop_back();
        }
    }
}
void TaskPlanner::printPath(const std::vector<int> &pathVisited, int currentProfit)
{
    std::cout << "Path visited: ";
    for (int node : pathVisited)
    {
        std::cout << node << " -> ";
    }
    std::cout << "Profit: " << currentProfit << std::endl;
}

std::pair<int, std::vector<int>> TaskPlanner::findMaxProfit()
{
    int maxProfit = 0;
    std::vector<bool> visited(num_nodes, false);
    std::vector<int> bestPath;
    int shortestPath;
    std::vector<int> pathVisited;

    // Start from the first node
    visited[0] = true;
    pathVisited.push_back(0);
    calculatePath(0, 0.0f, profits[0], visited, maxProfit, pathVisited, bestPath, shortestPath);

    return {maxProfit, bestPath};
}

void TaskPlanner::graphCallback(const interfaces::msg::Graph::SharedPtr msg)
{
    if (graph_received)
        return;
    graph_received = true;

    num_nodes = msg->num_nodes;
    travel_distances = msg->travel_distance;
    profits = msg->profits;

    RCLCPP_INFO(this->get_logger(), "Received Graph message");
    if (graph_received && tmax_received)
        onAllReceived();
}

void TaskPlanner::onAllReceived() {
    auto result = this->findMaxProfit();

    int maxProfit = result.first;
    std::vector<int> bestPath = result.second;

    auto resultTP_msg = interfaces::msg::Result();
    for (size_t i = 0; i < bestPath.size(); i++)
    {
        resultTP_msg.nodes_visited.push_back(static_cast<int>(bestPath[i]));
    }
    resultTP_msg.profit = maxProfit;
    RCLCPP_INFO(this->get_logger(), "Publishing resultTP");
    result_publisher->publish(resultTP_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
