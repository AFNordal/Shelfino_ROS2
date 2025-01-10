#pragma once
/*
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "planning/mapgeometry.hpp"
#include "planning/utils.hpp"
#include "planning/graphs.hpp"
#include "taskplanning/Matrix.hpp"
#include "vector"
#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <cstdlib>

#include "ortools/base/logging.h"
#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model.pb.h"
#include "ortools/sat/cp_model_solver.h"
#include "ortools/sat/model.h"
#include "ortools/sat/sat_parameters.pb.h"
#include "ortools/util/sorted_interval_list.h"


class Orienteering{
    public:
        std::vector<int> solve();
};

*/

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

