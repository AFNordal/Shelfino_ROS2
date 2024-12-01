#pragma once
#include <stdlib.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>

#include "matplotlibcpp/matplotlibcpp.h"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"


typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_2;
typedef K::Circle_2 Circle_2;
typedef K::Weighted_point_2 Weighted_point_2;
typedef K::Ray_2 Ray_2;
typedef K::Direction_2 Direction_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Bbox_2 Bbox_2;

namespace plt = matplotlibcpp;

Polygon_2 polygonROS2CGAL(geometry_msgs::msg::Polygon &msg);
Direction_2 orientation2dir(geometry_msgs::msg::Quaternion &orientation);
Polygon_2 circle2poly(Circle_2 &c, int n);
void draw_circle(const Point_2 &center, const double r, std::string color,  bool fill=false, int resolution=90);
void draw_poly(const Polygon_2 &p, std::string color,  bool fill=false);
bool point_in_bbox(const Point_2& p, const Bbox_2& box);

typedef enum
{
    CIRCLE,
    POLYGON
} ObstacleType;

class Obstacle
{
private:
    ObstacleType t;
    Polygon_2 polygon;
    Circle_2 circle;
    double radius;

public:
    Obstacle(obstacles_msgs::msg::ObstacleMsg &o);
    Polygon_2 getPoly() { return (t == CIRCLE) ? circle2poly(circle, 90) : polygon; };
    bool contains(const Point_2 &p);
};

class Map
{
private:
    Polygon_2 border;
    std::vector<Obstacle> obstacles;
    std::vector<Weighted_point_2> victims;
    Ray_2 shelfino;
    Ray_2 gate;
    double shelfino_r;

public:
    Map() {};
    void setBorder(geometry_msgs::msg::Polygon &p) { border = polygonROS2CGAL(p); };
    void setObstacles(std::vector<Obstacle> &o) { obstacles = o; };
    void setVictims(std::vector<Weighted_point_2> &v) { victims = v; };
    void setShelfinoInitPose(Ray_2 &pose) { shelfino = pose; };
    void setGatePose(Ray_2 &pose) { gate = pose; };
    void setShelfinoRadius(double r) { shelfino_r = r; }
    void display();
    Bbox_2 getBbox();
    bool isFree(const Point_2 &p);
};

