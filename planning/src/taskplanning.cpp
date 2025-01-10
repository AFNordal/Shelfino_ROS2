
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

void TaskPlanner::calculatePath() { //dynamic programming by gpt
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


/*
using namespace operations_research;
using namespace sat;
 
 std::vector<int> Orienteering::solve(){

    
    std::vector<int> result;
    CpModelBuilder cp_model;

    const int N = 5; // Number of nodes 
    const int Tmax = 10; // Maximum time budget
    const std::vector<int> profits = {0, 10, 20, 30, 0}; // Profit for each node
    const std::vector<std::vector<int>> travel_time = {
      {0, 2, 4, 6, 8},
      {2, 0, 2, 4, 6},
      {4, 2, 0, 2, 4},
      {6, 4, 2, 0, 2},
      {8, 6, 4, 2, 0}}; 

    // Decision variables
    std::vector<std::vector<IntVar>> x(N, std::vector<IntVar>(N));
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
        x[i][j] = cp_model.NewBoolVar().WithName("x_" + std::to_string(i) + "_" + std::to_string(j));
        }
    }

     // Objective: Maximize profits
    LinearExpr objective;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
        objective += profits[j] * x[i][j];
        }
    }
    cp_model.Maximize(objective);

      // Constraints
    // Start from node 1 and end at node N
    cp_model.AddEquality(Sum(x[0]), 1);
    cp_model.AddEquality(Sum(x[N - 1]), 1);

        // Each node can be visited at most once 
        for (int k = 1; k < N - 1; ++k) {
        LinearExpr incoming;
        LinearExpr outgoing;

            for (int i = 1; i < N-1; ++i) {
                incoming += x[i][k];  // Sum of edges entering node k
            }
            for (int j = 1; j < N-1; ++j) {
                outgoing += x[k][j];  // Sum of edges leaving node k
            }

            // Ensure that incoming and outgoing edges are consistent
            cp_model.AddEquality(incoming, outgoing);

            // Ensure a node is visited at most once
            cp_model.AddLessOrEqual(incoming, 1);
        }
        // Total travel time should not exceed Tmax
        LinearExpr total_time;
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
            total_time += travel_time[i][j] * x[i][j];
            }
        }
        cp_model.AddLessOrEqual(total_time, Tmax);

// Subtour elimination
     std::vector<IntVar> u(N);
    for (int i = 1; i < N; ++i) {
        u[i] = cp_model.NewIntVar({1, N - 1}).WithName("u_" + std::to_string(i));
    }
    for (int i = 1; i < N; ++i) {
        for (int j = 1; j < N; ++j) {
        if (i != j) {
            cp_model.AddLessOrEqual(u[i] - u[j] + N * x[i][j], N - 1);
        }
        }
    }
    // Solve the model
  const CpSolverResponse response = Solve(cp_model.Build());

  // Print the solution
  if (response.status() == CpSolverStatus::OPTIMAL || response.status() == CpSolverStatus::FEASIBLE) {
    std::cout << "Objective value: " << response.objective_value() << std::endl;
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j < N; ++j) {
        if (SolutionIntegerValue(response, x[i][j]) == 1) {
          std::cout << "Travel from " << i << " to " << j << std::endl;
          result.push_back(j)
        }
      }
    }
  } else {
    std::cout << "No solution found." << std::endl;
  }
  return result
}


int main() {
    std::vector<int> vec = operations_research::sat::solve();
    std::cout << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i != vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/Graph.hpp"

class GraphSubscriber : public rclcpp::Node {
public:
    GraphSubscriber()
        : Node("graph_subscriber") {
        auto graph_subscriber_ = this->create_subscription<interfaces::msg::Graph>(
            "/graph_topic",      // Topic name
            10,                  // Queue size
            std::bind(&GraphSubscriber::graphCallback, this, std::placeholders::_1)
        );
    }

private:
    void graphCallback(const interfaces::msg::Graph::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received graph with");

        // Access the profits
        for (size_t i = 0; i < msg->profits.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Profit for node");
        }

        // Access the travel distances (flattened adjacency matrix)
        int num_nodes = msg->num_nodes;
        for (int i = 0; i < num_nodes; ++i) {
            for (int j = 0; j < num_nodes; ++j) {
                int index = i * num_nodes + j;
                RCLCPP_INFO(this->get_logger(), "Distance");
            }
        }
    }

    rclcpp::Subscription<interfaces::msg::Graph>::SharedPtr graph_subscriber_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphSubscriber>());
    rclcpp::shutdown();
    return 0;
}*/ 

