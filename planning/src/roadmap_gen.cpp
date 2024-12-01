#include "roadmap_gen.hpp"

RoadmapGenerator::RoadmapGenerator()
    : Node("roadmap_gen")
{
    map = Map();
    rclcpp::QoS TL_qos(rclcpp::KeepLast(1));
    TL_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    rclcpp::QoS V_qos(rclcpp::KeepLast(1));
    V_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    borderSubscription = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/borders", TL_qos, std::bind(&RoadmapGenerator::border_callback, this, _1));
    obstaclesSubscription = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", TL_qos, std::bind(&RoadmapGenerator::obstacles_callback, this, _1));
    victimsSubscription = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/victims", TL_qos, std::bind(&RoadmapGenerator::victims_callback, this, _1));
    initPoseSubscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino1/initialpose", V_qos, std::bind(&RoadmapGenerator::initPose_callback, this, _1));
    shelfinoDescrSubscription = this->create_subscription<std_msgs::msg::String>(
        "/shelfino1/robot_description", TL_qos, std::bind(&RoadmapGenerator::shelfinoDescr_callback, this, _1));
    gateSubscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gates_position", TL_qos, std::bind(&RoadmapGenerator::gate_callback, this, _1));
}

void RoadmapGenerator::border_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received border with %ld verteces", msg->polygon.points.size());
    map.setBorder(msg->polygon);
    border_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received %ld obstacles", msg->obstacles.size());
    std::vector<Obstacle> obstacles;
    for (auto o : msg->obstacles)
    {
        obstacles.emplace_back(o);
    }
    map.setObstacles(obstacles);
    obstacles_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::victims_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received %ld victims", msg->obstacles.size());
    std::vector<Weighted_point_2> victims;
    for (auto o : msg->obstacles)
    {
        auto p = o.polygon.points.at(0);
        victims.emplace_back(Point_2(p.x, p.y), o.radius);
    }
    map.setVictims(victims);
    victims_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::initPose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received initial shelfino pose");
    auto orientation = msg->pose.pose.orientation;
    auto pos = msg->pose.pose.position;
    Direction_2 dir = orientation2dir(orientation);
    Ray_2 CGALPose = Ray_2(Point_2(pos.x, pos.y), dir);
    map.setShelfinoInitPose(CGALPose);

    initPose_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::shelfinoDescr_callback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received shelfino description");

    double rad = parseShelfinoRadius(msg->data);
    map.setShelfinoRadius(rad);
    shelfinoDescr_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::gate_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received %ld gate(s)", msg->poses.size());
    auto orientation = msg->poses.at(0).orientation;
    auto pos = msg->poses.at(0).position;
    Direction_2 dir = orientation2dir(orientation);
    Ray_2 CGALPose = Ray_2(Point_2(pos.x, pos.y), dir);
    map.setGatePose(CGALPose);

    initPose_received = true; // WORKAROUND
    Ray_2 dummyShelfino{Point_2{0, 0}, Direction_2{1, 0}};
    map.setShelfinoInitPose(dummyShelfino);
    gate_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::on_map_complete()
{
    RCLCPP_INFO(this->get_logger(), "Received all map ingredients");
    Bbox_2 bbox = map.getBbox();
    const size_t N = 5000;
    double *samples = hammersley_sequence(0, N - 1, 2, N);
    // std::vector<Point_2> inside;
    Graph G;
    for (size_t i = 0; i < N; i++)
    {
        Vertex q{
            samples[i * 2] * bbox.x_span() + bbox.xmin(),
            samples[i * 2 + 1] * bbox.y_span() + bbox.ymin()};
        if (map.isFree(q))
            G.addVertex(std::make_shared<Vertex>(q));
        
        // inside.push_back(q);
    }
    //  For each q in G.V
    //      For each n in q.KNN
    //          if not path(q, n) collides
    //              q.addNeighbour(n)
    //              n.addNeighbour(q)

    
    map.display();
    // auto coord_pair_in = points2coords(inside);
    // plt::scatter(coord_pair_in.first, coord_pair_in.second, 1, {{"color", "g"}});
    plt::show();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoadmapGenerator>());
    rclcpp::shutdown();
    return 0;
}