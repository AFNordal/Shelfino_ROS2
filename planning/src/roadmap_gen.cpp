#include "roadmap_gen.hpp"

RoadmapGenerator::RoadmapGenerator()
    : Node("roadmap_gen")
{
    map = Map();
    rclcpp::QoS TL_qos(rclcpp::KeepLast(1));
    TL_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    // rclcpp::QoS V_qos(rclcpp::KeepLast(1));
    // V_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
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
}

void RoadmapGenerator::border_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
    // mtx.lock();
    RCLCPP_INFO(this->get_logger(), "Received border with %ld verteces", msg->polygon.points.size());
    map.setBorder(msg->polygon);
    border_received = true;
    if (received_all() && (!mapping_started))
    {
        mapping_started = true;
        // mtx.unlock();
        on_map_complete();
    }
    else
    {
        // mtx.unlock();
    }
}

void RoadmapGenerator::dummy_border()
{
    printf("Dummy border\n");
    geometry_msgs::msg::Polygon polygon;

    // Create and define four vertices
    geometry_msgs::msg::Point32 vertex1;
    vertex1.x = 0.0;
    vertex1.y = 0.0;
    vertex1.z = 0.0;

    geometry_msgs::msg::Point32 vertex2;
    vertex2.x = 100.0;
    vertex2.y = 0.0;
    vertex2.z = 0.0;

    geometry_msgs::msg::Point32 vertex3;
    vertex3.x = 100.0;
    vertex3.y = 100.0;
    vertex3.z = 0.0;

    geometry_msgs::msg::Point32 vertex4;
    vertex4.x = 0.0;
    vertex4.y = 100.0;
    vertex4.z = 0.0;

    // Add the vertices to the polygon
    polygon.points.push_back(vertex1);
    polygon.points.push_back(vertex2);
    polygon.points.push_back(vertex3);
    polygon.points.push_back(vertex4);

    map.setBorder(polygon);
    border_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    // mtx.lock();
    RCLCPP_INFO(this->get_logger(), "Received %ld obstacles", msg->obstacles.size());
    std::vector<Obstacle> obstacles;
    for (auto o : msg->obstacles)
    {
        obstacles.emplace_back(o);
    }
    map.setObstacles(obstacles);
    obstacles_received = true;
    if (received_all() && (!mapping_started))
    {
        mapping_started = true;
        // mtx.unlock();
        on_map_complete();
    }
    else
    {
        // mtx.unlock();
    }
}

void RoadmapGenerator::dummy_obstacles()
{
    printf("Dummy obst\n");
    std::vector<Obstacle> obstacles;
    obstacles_msgs::msg::ObstacleMsg o{};
    geometry_msgs::msg::Polygon pol{};
    geometry_msgs::msg::Point32 p{};
    p.x = 10;
    p.y = 10;
    p.z = 0;
    pol.points.push_back(p);
    o.set__radius(2);
    o.set__polygon(pol);
    obstacles.emplace_back(o);
    map.setObstacles(obstacles);

    obstacles_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::victims_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    // mtx.lock();
    RCLCPP_INFO(this->get_logger(), "Received %ld victims", msg->obstacles.size());
    std::vector<Weighted_point_2> victims;
    for (auto o : msg->obstacles)
    {
        auto p = o.polygon.points.at(0);
        victims.emplace_back(Point_2(p.x, p.y), o.radius);
    }
    map.setVictims(victims);
    victims_received = true;
    if (received_all() && (!mapping_started))
    {
        mapping_started = true;
        // mtx.unlock();
        on_map_complete();
    }
    else
    {
        // mtx.unlock();
    }
}

void RoadmapGenerator::dummy_victims()
{
    printf("Dummy victims\n");
    std::vector<Weighted_point_2> victims;
    victims.emplace_back(Point_2(50, 10), 550);
    map.setVictims(victims);
    victims_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::initPose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // mtx.lock();
    RCLCPP_INFO(this->get_logger(), "Received initial shelfino pose");
    auto orientation = msg->pose.pose.orientation;
    auto pos = msg->pose.pose.position;
    Direction_2 dir = orientation2dir(orientation);
    Ray_2 CGALPose = Ray_2(Point_2(pos.x, pos.y), dir);
    map.setShelfinoInitPose(CGALPose);

    initPose_received = true;
    if (received_all() && (!mapping_started))
    {
        mapping_started = true;
        // mtx.unlock();
        on_map_complete();
    }
    else
    {
        // mtx.unlock();
    }
}

void RoadmapGenerator::dummy_initPose()
{
    printf("Dummy initpose\n");
    geometry_msgs::msg::Pose::_orientation_type orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;
    orientation.w = 1.0;

    Direction_2 dir = orientation2dir(orientation);
    Ray_2 CGALPose = Ray_2(Point_2(20, 80), dir);
    map.setShelfinoInitPose(CGALPose);

    initPose_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::shelfinoDescr_callback(std_msgs::msg::String::SharedPtr msg)
{
    // mtx.lock();
    RCLCPP_INFO(this->get_logger(), "Received shelfino description");
    double rad = parseShelfinoRadius(msg->data);
    RCLCPP_INFO(this->get_logger(), "Parsed shelfino description");
    map.setShelfinoRadius(rad);
    shelfinoDescr_received = true;
    if (received_all() && (!mapping_started))
    {
        mapping_started = true;
        // mtx.unlock();
        on_map_complete();
    }
    else
    {
        // mtx.unlock();
    }
}

void RoadmapGenerator::dummy_shelfinoDescr()
{
    printf("Dummy descr\n");

    map.setShelfinoRadius(1);
    shelfinoDescr_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::gate_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // mtx.lock();
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
    if (received_all() && (!mapping_started))
    {
        mapping_started = true;
        // mtx.unlock();
        on_map_complete();
    }
    else
    {
        // mtx.unlock();
    }
}

void RoadmapGenerator::dummy_gate()
{
    printf("Dummy gate\n");
    geometry_msgs::msg::Pose::_orientation_type orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;
    orientation.w = 1.0;

    Direction_2 dir = orientation2dir(orientation);
    Ray_2 CGALPose = Ray_2(Point_2(60, 30), dir);
    map.setGatePose(CGALPose);

    gate_received = true;
    if (received_all())
    {
        on_map_complete();
    }
}

void RoadmapGenerator::on_map_complete()
{
    printf("Received all map ingredients\n");
    Bbox_2 bbox = map.getBbox();
    const size_t N = 5000;
    double *samples = hammersley_sequence(0, N - 1, 2, N);
    printf("Got samples\n");

    map.offsetAllPolys();


    Graph G;
    std::vector<shared_ptr<Vertex>> POIs;
    std::shared_ptr<Vertex> gateVertex = std::make_shared<Vertex>(map.getGate().source());
    G.addVertex(gateVertex);
    POIs.push_back(gateVertex);
    std::shared_ptr<Vertex> shelfinoVertex = std::make_shared<Vertex>(map.getShelfino().source());
    G.addVertex(shelfinoVertex);
    POIs.push_back(shelfinoVertex);
    for (auto v : map.getVictims())
    {
        std::shared_ptr<Vertex> victimVertex = std::make_shared<Vertex>(v.point());
        G.addVertex(victimVertex);
        POIs.push_back(victimVertex);
    }
    for (size_t i = 0; i < N; i++)
    {
        Vertex q{
            samples[i * 2] * bbox.x_span() + bbox.xmin(),
            samples[i * 2 + 1] * bbox.y_span() + bbox.ymin()};
        if (map.isFree(q) && !map.isPOI(q))
            G.addVertex(std::make_shared<Vertex>(q));
    }
    printf("inserted verteces\n");

    map.display();

    for (auto &q : *G.getVerteces())
    {
        auto knn = G.KNN(q, KNN_K);
        for (auto &n : *knn)
        {
            if (q == n) // don't connect to itself
                continue;
            bool already_connected = false;
            for (auto n_n : *(n->getNeighbours()))
            {
                if (n_n.first == q)
                { // don't connect if already connected
                    already_connected = true;
                    break;
                }
            }
            if (already_connected)
                continue;
            Segment_2 seg{*q, *n};
            if (map.isFree(seg))
            {
                G.connect(q, n, std::make_shared<SegmentEdge>(seg));
            }
        }
    }
    printf("inserted edges\n");
    for (size_t i=0; i<POIs.size(); i++) {
        for (size_t j=i+1; j<POIs.size(); j++) {

            std::vector<std::shared_ptr<Vertex>> shortestPath = G.dijkstra(POIs.at(i), POIs.at(j));
            for (size_t k = 0; k < shortestPath.size()-1; k++) {
                auto p1 = shortestPath.at(k);
                auto p2 = shortestPath.at(k+1);
                draw_segment(Segment_2(*p1, *p2), "m");
            }

        } 
    }
    for (auto e : *G.getEdges())
    {
        draw_segment(e->getSegment(), "k", 0.5);
    }

    delete[] samples;
    plt_show();
}

int main(int argc, char *argv[])
{
    // RoadmapGenerator rmg{};
    // rmg.dummy_border();
    // rmg.dummy_obstacles();
    // rmg.dummy_victims();
    // rmg.dummy_initPose();
    // rmg.dummy_shelfinoDescr();
    // rmg.dummy_gate();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoadmapGenerator>());
    rclcpp::shutdown();
    return 0;
}