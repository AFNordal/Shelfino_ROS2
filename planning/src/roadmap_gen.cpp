#include "roadmap_gen.hpp"

RoadmapGenerator::RoadmapGenerator()
    : Node("roadmap_gen")
{
    this->declare_parameter("strategy", "combinatorial");
    this->declare_parameter("skip_shelfino", false);
    std::string strategyStr = this->get_parameter("strategy").as_string();
    skip_shelfino = this->get_parameter("skip_shelfino").as_bool();
    if (strategyStr == "combinatorial")
    {
        strategy = COMBINATORIAL;
    }
    else
    {
        strategy = PROBABILISTIC;
    }
    draw_points({}, "r");
    plt_show(4);
    map = Map();
    rclcpp::QoS TL_qos(rclcpp::KeepLast(1));
    TL_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    borderSubscription = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/borders", TL_qos, std::bind(&RoadmapGenerator::border_callback, this, _1));
    obstaclesSubscription = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", TL_qos, std::bind(&RoadmapGenerator::obstacles_callback, this, _1));
    victimsSubscription = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/victims", TL_qos, std::bind(&RoadmapGenerator::victims_callback, this, _1));
    initPoseSubscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino/amcl_pose", TL_qos, std::bind(&RoadmapGenerator::initPose_callback, this, _1));
    shelfinoDescrSubscription = this->create_subscription<std_msgs::msg::String>(
        "/shelfino/robot_description", TL_qos, std::bind(&RoadmapGenerator::shelfinoDescr_callback, this, _1));
    gateSubscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gates", TL_qos, std::bind(&RoadmapGenerator::gate_callback, this, _1));
    graphPublisher = this->create_publisher<interfaces::msg::Graph>("/graph_topic", TL_qos);
    TPResultSubscription = this->create_subscription<interfaces::msg::Result>(
        "/resultTP_topic", TL_qos, std::bind(&RoadmapGenerator::TPResult_callback, this, _1));
    tmax_subscription = this->create_subscription<std_msgs::msg::Int32>(
        "/victims_timeout", TL_qos,
        std::bind(&RoadmapGenerator::tmax_callback, this, std::placeholders::_1));
    pathPublisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/planned_path", TL_qos);
}

void RoadmapGenerator::tmax_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (tmax_received)
        return;
    tmax_received = true;
    tmax = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received tmax %f", tmax);
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::TPResult_callback(interfaces::msg::Result::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received path with profit %d:", msg->profit);

    double th0 = dir2ang(map.getShelfino().direction());
    double th1 = dir2ang(map.getGate().direction());
    std::vector<Point_2> pathPoints{};
    std::vector<int> nodeIndicesInPath{};
    pathPoints.push_back(map.getShelfino().source());
    nodeIndicesInPath.push_back(0);
    auto POIs = map.getPOIs();
    for (size_t i = 0; i < msg->nodes_visited.size() - 1; i++)
    {
        int nodeIdx = msg->nodes_visited.at(i);
        int nextNodeIdx = msg->nodes_visited.at(i + 1);
        auto segPathPoints = pathPointMatrix.at(nextNodeIdx).at(nodeIdx);
        for (size_t j = 1; j < segPathPoints.size(); j++)
            pathPoints.push_back(segPathPoints.at(j));
        nodeIndicesInPath.push_back(pathPoints.size() - 1);
    }

    RCLCPP_INFO(this->get_logger(), "Calculating dubins path through %ld vertices", pathPoints.size());

    std::vector<SPDubinsPath> sol{};
    std::vector<int> collisions;
    auto MPResult = optimalMPDubinsParams(sol, pathPoints, th0, th1,
                                          1. / SHELFINO_TURNING_R, 16,
                                          true, true, map, collisions);

    double req_time = MPResult.first / SHELFINO_VEL;
    RCLCPP_INFO(this->get_logger(), "Found dubins path that takes %f seconds", req_time);

    plt_clear();
    map.display();
    for (size_t i = 0; i < pathPoints.size() - 1; i++)
    {
        auto PL = sol.at(i).getPolyline(90);
        draw_polyline(PL, "r");
    }
    plt_draw();

    if (req_time > tmax)
    {
        tmax = msg->time - 0.1;
        RCLCPP_INFO(this->get_logger(), "Path is too long. Retrying with tmax=%f", tmax);
        sendGraph();
        return;
    }

    for (size_t i = 0; i < collisions.size(); i++)
    {
        if ((i == collisions.size() - 1) && !map.isWithinBorder(map.getGate().source()))
            collisions.at(i)--; // If gate lies outside of map, one collision is expected
        if (collisions.at(i) > 0)
        {
            for (size_t j = 0; j < msg->nodes_visited.size(); j++)
            {
                if (nodeIndicesInPath.at(j) >= i + 1)
                {
                    int sourceNode = msg->nodes_visited.at(j - 1);
                    int targetNode = msg->nodes_visited.at(j);
                    RCLCPP_INFO(this->get_logger(), "Collision between node %d and %d. Modifying distance matrix and retrying", sourceNode, targetNode);
                    distMatrix.at(sourceNode).at(targetNode) = INFINITY;
                    distMatrix.at(targetNode).at(sourceNode) = INFINITY;
                    sendGraph();
                    return;
                }
            }
        }
    }

    std::vector<Point_2> pointApprox;
    for (size_t i = 0; i < pathPoints.size() - 1; i++)
    {
        std::vector<Point_2> p = sol.at(i).getPointApprox(0.1);
        pointApprox.insert(pointApprox.end(), p.begin(), p.end());
    }
    pointApprox.push_back(sol.back().getEndPoint());
    draw_points(pointApprox, "g", 3);
    plt_draw();
    RCLCPP_INFO(this->get_logger(), "Final path found");

    geometry_msgs::msg::PoseArray pathMsg;
    for (size_t i = 1; i < pointApprox.size(); i++)
    {
        Point_2 pt = pointApprox.at(i);
        geometry_msgs::msg::Pose ps;
        auto dir = pointApprox.at(i) - pointApprox.at(i - 1);
        tf2::Quaternion q{};
        double yaw = std::atan2(dir.y(), dir.x());
        q.setRPY(0, 0, yaw);
        ps.position.x = pt.x();
        ps.position.y = pt.y();
        ps.position.z = 0;
        ps.orientation = tf2::toMsg(q);
        pathMsg.poses.push_back(ps);
    }
    pathPublisher->publish(pathMsg);
    plt_show();
}

// Convert to message
void RoadmapGenerator::sendGraph(void)
{

    auto graph_msg = interfaces::msg::Graph();

    // Set the number of nodes
    graph_msg.num_nodes = map.getVictims().size() + 2;
    size_t num_nodes = map.getVictims().size() + 2;
    // Set the profits for each node
    graph_msg.profits.push_back(0); // first node is init_pos
    for (const auto &victim : map.getVictims())
    {
        graph_msg.profits.push_back(victim.weight());
    }
    graph_msg.profits.push_back(0); // last node is gate

    // Flatten the adjacency matrix for travel distances
    for (size_t i = 0; i < num_nodes; i++)
    {
        for (size_t j = 0; j < num_nodes; j++)
        {
            graph_msg.travel_distance.push_back(distMatrix.at(i).at(j));
        }
    }
    graph_msg.tmax = tmax;

    RCLCPP_INFO(this->get_logger(), "Sending graph to task planner");
    graphPublisher->publish(graph_msg);
}

void RoadmapGenerator::border_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
    if (border_received)
        return;
    RCLCPP_INFO(this->get_logger(), "Received border with %ld vertices", msg->polygon.points.size());
    map.setBorder(msg->polygon);
    map.draw_border();
    plt_draw();
    border_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    if (obstacles_received)
        return;
    RCLCPP_INFO(this->get_logger(), "Received %ld obstacles", msg->obstacles.size());
    std::vector<Obstacle> obstacles;
    for (auto o : msg->obstacles)
    {
        obstacles.emplace_back(o);
    }
    map.setObstacles(obstacles);
    map.draw_obstacles();
    plt_draw();
    obstacles_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::victims_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    if (victims_received)
        return;
    RCLCPP_INFO(this->get_logger(), "Received %ld victims", msg->obstacles.size());
    std::vector<Weighted_point_2> victims;
    for (auto o : msg->obstacles)
    {
        auto p = o.polygon.points.at(0);
        victims.emplace_back(Point_2(p.x, p.y), o.radius);
    }
    map.setVictims(victims);
    map.draw_victims();
    plt_draw();
    victims_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::initPose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (initPose_received)
        return;
    RCLCPP_INFO(this->get_logger(), "Received initial shelfino pose");
    auto orientation = msg->pose.pose.orientation;
    auto pos = msg->pose.pose.position;
    Direction_2 dir = orientation2dir(orientation);
    Ray_2 CGALPose = Ray_2(Point_2(pos.x, pos.y), dir);
    map.setShelfinoInitPose(CGALPose);
    if (shelfinoDescr_received)
    {
        map.draw_shelfino();
        plt_draw();
    }

    initPose_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::shelfinoDescr_callback(std_msgs::msg::String::SharedPtr msg)
{
    if (shelfinoDescr_received)
        return;
    RCLCPP_INFO(this->get_logger(), "Received shelfino description");
    double rad = parseShelfinoRadius(msg->data);
    RCLCPP_INFO(this->get_logger(), "Parsed shelfino description");
    map.setShelfinoRadius(rad);
    if (initPose_received)
    {
        map.draw_shelfino();
        plt_draw();
    }
    shelfinoDescr_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::gate_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (gate_received)
        return;
    RCLCPP_INFO(this->get_logger(), "Received %ld gate(s)", msg->poses.size());
    auto orientation = msg->poses.at(0).orientation;
    auto pos = msg->poses.at(0).position;
    Direction_2 dir = orientation2dir(orientation);
    Ray_2 CGALPose = Ray_2(Point_2(pos.x, pos.y), dir);
    map.setGatePose(CGALPose);
    map.draw_gate();
    plt_draw();

    if (skip_shelfino)
    {
        RCLCPP_INFO(this->get_logger(), "Found skip_shelfino=true, creating dummy");
        initPose_received = true;
        Ray_2 dummyShelfino{Point_2{0, 0}, Direction_2{1, 0}};
        map.setShelfinoInitPose(dummyShelfino);
    }

    gate_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::smooth_bisect(const std::vector<std::shared_ptr<Vertex>> &path,
                                     std::list<std::shared_ptr<Vertex>> &smooth,
                                     size_t idx0, size_t idx1,
                                     const std::list<std::shared_ptr<Vertex>>::iterator &smooth_inserter)
{
    if (idx0 + 1 == idx1)
    {
        return;
    }
    if (map.isFree(Segment_2(path.at(idx0)->getAlias(), path.at(idx1)->getAlias())))
    {
        return;
    }
    size_t mid_idx = (idx1 + idx0) / 2;
    auto first_half_inserter = smooth.insert(smooth_inserter, path.at(mid_idx));
    smooth_bisect(path, smooth, mid_idx, idx1, smooth_inserter);
    smooth_bisect(path, smooth, idx0, mid_idx, first_half_inserter);
}

// Create a graph of vertices that are Hammersley samples, and connect them in a PRM.
// G is a Graph object that already contains the POI vertices.
void RoadmapGenerator::generate_PRM(Graph &G)
{
    Bbox_2 bbox = map.getBbox();
    double *samples = hammersley_sequence(0, N_SAMPLES - 1, 2, N_SAMPLES);
    RCLCPP_INFO(this->get_logger(), "Got Hammersley samples");

    // Insert samples that are free as vertices in G
    for (size_t i = 0; i < N_SAMPLES; i++)
    {
        Vertex q{
            samples[i * 2] * bbox.x_span() + bbox.xmin(),
            samples[i * 2 + 1] * bbox.y_span() + bbox.ymin()};
        if (map.isFree(q) && !map.isPOI(q))
            G.addVertex(std::make_shared<Vertex>(q));
    }

    // Connect nearby samples if the segment between them is free
    for (auto &q : *G.getVertices())
    {
        auto knn = G.KNN(q, KNN_K);
        for (auto &n : *knn)
        {
            // don't connect a vertex to itself
            if (q == n)
                continue;

            // don't connect vertices that are already neighbours
            bool already_connected = false;
            for (auto n_n : *(n->getNeighbours()))
            {
                if (n_n.first == q)
                {
                    already_connected = true;
                    break;
                }
            }
            if (already_connected)
                continue;

            Segment_2 aliasSeg{q->getAlias(), n->getAlias()};
            if (map.isFree(aliasSeg))
                G.connect(q, n, std::make_shared<SegmentEdge>(Segment_2{*q, *n}));
        }
    }
    RCLCPP_INFO(this->get_logger(), "PRM created");
    delete[] samples;
}

double realFmod(double x, double y)
{
    if (x < 0)
        return y - fmod(-x, y);
    else
        return fmod(x, y);
}

bool withinModPi(double x, double l, double u)
{

    x = realFmod(x, M_PI);
    l = realFmod(l, M_PI);
    u = realFmod(u, M_PI);
    if (u >= l)
    {
        return (x >= l && x <= u);
    }
    else
    {
        return (x >= l || x <= u);
    }
}

void RoadmapGenerator::minimal_clearance_graph(Graph &G, const std::vector<shared_ptr<Vertex>> &POIs)
{
    // Graph G;
    std::vector<Point_2> offsetRV;
    std::vector<double> inAngs, outAngs;
    std::vector<Point_2> rv = map.getReflexVertices(offsetRV, inAngs, outAngs);
    std::vector<shared_ptr<Vertex>> addedVertices;
    size_t N_rv = offsetRV.size();
    for (size_t i = 0; i < N_rv; i++)
    {
        auto v = std::make_shared<Vertex>(offsetRV.at(i));
        addedVertices.push_back(v);
        G.addVertex(v);
    }
    // auto vertices = G.getVertices();
    draw_points(rv, "r", 3);
    plt_draw();
    for (size_t i = 0; i < N_rv; i++)
    {
        auto visPoints = map.visibilityQuery(offsetRV.at(i));
        for (auto &vp : visPoints)
        {
            for (size_t j = 0; j < N_rv; j++)
            {
                if ((rv.at(j) == vp) && (j != i))
                {
                    Segment_2 seg{rv.at(i), rv.at(j)};
                    double ang = dir2ang(seg.direction());
                    if (withinModPi(ang, inAngs[i], outAngs[i]) &&
                        withinModPi(ang, inAngs[j], outAngs[j]))
                    {
                        G.connect(addedVertices.at(i), addedVertices.at(j),
                                  std::make_shared<SegmentEdge>(Segment_2{*addedVertices.at(i), *addedVertices.at(j)}));
                        // draw_segment(Segment_2{*addedVertices.at(i), *addedVertices.at(j)}, "orange");
                        // draw_segment(seg, "red");
                    }
                }
            }
        }
    }

    // Connect POIs to eachother
    for (size_t i = 0; i < POIs.size(); i++)
    {
        for (size_t j = 0; j < i; j++)
        {
            Segment_2 aliasSeg{POIs.at(i)->getAlias(), POIs.at(j)->getAlias()};
            if (map.isFree(aliasSeg))
            {
                G.connect(POIs.at(i), POIs.at(j),
                          std::make_shared<SegmentEdge>(Segment_2{*POIs.at(i), *POIs.at(j)}));
                // draw_segment(Segment_2{*POIs.at(i), *POIs.at(j)}, "b");
            }
        }
    }
    for (auto &p : POIs)
    {
        std::vector<Point_2> visPoints;
        visPoints = map.visibilityQuery(p->getAlias());
        for (auto &vp : visPoints)
        {
            for (size_t i = 0; i < N_rv; i++)
            {
                if ((rv.at(i) == vp))
                {
                    Segment_2 seg{*p, *(addedVertices.at(i))};
                    double ang = dir2ang(seg.direction());
                    if (withinModPi(ang, inAngs[i], outAngs[i]))
                    {
                        G.connect(p, addedVertices.at(i), std::make_shared<SegmentEdge>(seg));
                        // draw_segment(seg, "k");
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Min. clearance graph done");
}

void RoadmapGenerator::paths_from_roadmap()
{
    // Initialize PRM graph and insert POI vertices
    Graph G;
    std::vector<shared_ptr<Vertex>> POIs;
    std::shared_ptr<Vertex> shelfinoVertex = std::make_shared<Vertex>(map.getShelfino().source());
    G.addVertex(shelfinoVertex);
    POIs.push_back(shelfinoVertex);
    for (auto v : map.getVictims())
    {
        std::shared_ptr<Vertex> victimVertex = std::make_shared<Vertex>(v.point());
        G.addVertex(victimVertex);
        POIs.push_back(victimVertex);
    }
    std::shared_ptr<Vertex> gateVertex = std::make_shared<Vertex>(map.getGate().source(), map.getGateProjection());
    G.addVertex(gateVertex);
    POIs.push_back(gateVertex);
    // Distance matrix for planner
    distMatrix = std::vector<std::vector<double>>(POIs.size(), std::vector<double>(POIs.size()));
    pathPointMatrix = std::vector<std::vector<std::vector<Point_2>>>(POIs.size(), std::vector<std::vector<Point_2>>(POIs.size(), std::vector<Point_2>{}));
    if (strategy == COMBINATORIAL)
        minimal_clearance_graph(G, POIs);
    else
        generate_PRM(G);

    int path_cntr = 0;
    for (size_t i = 0; i < POIs.size(); i++)
    {
        for (size_t j = i + 1; j < POIs.size(); j++)
        {
            std::vector<std::shared_ptr<Vertex>> shortestPath = G.dijkstra(POIs.at(i), POIs.at(j));
            std::shared_ptr<Vertex> first = shortestPath.front();
            std::shared_ptr<Vertex> last = shortestPath.back();

            std::vector<Point_2> pathPoints{};
            if (strategy == PROBABILISTIC)
            {
                std::list<std::shared_ptr<Vertex>> smoothedPath;
                smoothedPath.push_front(first);
                smoothedPath.push_back(last);
                smooth_bisect(shortestPath, smoothedPath, 0, shortestPath.size() - 1, --smoothedPath.end());

                for (auto &p : smoothedPath)
                    pathPoints.push_back(*p);
            }
            else
            {
                for (auto &p : shortestPath)
                    pathPoints.push_back(*p);
            }
            // draw_polyline(pathPoints, matplotlib_color_range[col_cntr]);
            std::vector<SPDubinsPath> sol;
            double th0, th1;
            bool th0_constrained, th1_constrained;
            bool gate_connection = false;
            if (first == shelfinoVertex)
            {
                th0 = dir2ang(map.getShelfino().direction());
                th0_constrained = true;
            }
            else if (first == gateVertex)
            {
                th0 = M_PI + dir2ang(map.getGate().direction());
                th0_constrained = true;
                gate_connection = true;
            }
            else
            {
                th0 = 0;
                th0_constrained = false;
            }

            if (last == shelfinoVertex)
            {
                th1 = M_PI + dir2ang(map.getShelfino().direction());
                th1_constrained = true;
            }
            else if (last == gateVertex)
            {
                th1 = dir2ang(map.getGate().direction());
                th1_constrained = true;
                gate_connection = true;
            }
            else
            {
                th1 = 0;
                th1_constrained = false;
            }
            std::vector<int> collision_locator;
            auto MPResult = optimalMPDubinsParams(sol, pathPoints, th0, th1,
                                                  1. / SHELFINO_TURNING_R, 16,
                                                  th0_constrained, th1_constrained, map, collision_locator);
            double L = MPResult.first / SHELFINO_VEL;
            int collisions = MPResult.second;
            if (gate_connection && !(map.isWithinBorder(map.getGate().source())))
                collisions--;
            if (collisions > 0)
                L = INFINITY;
            distMatrix.at(i).at(j) = L;
            distMatrix.at(j).at(i) = L;
            pathPointMatrix.at(i).at(j) = pathPoints;
            pathPointMatrix.at(j).at(i) = vector<Point_2>(pathPoints.rbegin(), pathPoints.rend());

            for (size_t i = 0; i < pathPoints.size() - 1; i++)
            {
                auto PL = sol.at(i).getPolyline(90);
                draw_polyline(PL, matplotlib_color_range[path_cntr % matplotlib_color_range.size()]);
            }
            draw_points(pathPoints, "b");
            plt_draw();
            path_cntr++;
            RCLCPP_INFO(this->get_logger(), "Found dubins path %d/%ld", path_cntr, (POIs.size() * (POIs.size() - 1)) / 2);
        }
    }
}

void RoadmapGenerator::on_map_complete()
{
    RCLCPP_INFO(this->get_logger(), "Received all map ingredients");
    map.offsetAllPolys();
    plt_clear();
    map.display();
    plt_draw();
    RCLCPP_INFO(this->get_logger(), "Map polygons offset");
    paths_from_roadmap();
    sendGraph();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoadmapGenerator>());
    rclcpp::shutdown();

    return 0;
}
