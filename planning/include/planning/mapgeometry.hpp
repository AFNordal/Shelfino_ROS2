#pragma once
#include <stdlib.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>

#include "matplotlibcpp/matplotlibcpp.h"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"



// typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Circle_2 Circle_2;
typedef K::Weighted_point_2 Weighted_point_2;
typedef K::Segment_2 Segment_2;
typedef K::Ray_2 Ray_2;
typedef K::Direction_2 Direction_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Bbox_2 Bbox_2;

namespace plt = matplotlibcpp;

constexpr std::array<const char*, 10> matplotlib_color_range = {
    "#1f77b4", // Blue
    "#ff7f0e", // Orange
    "#2ca02c", // Green
    "#d62728", // Red
    "#9467bd", // Purple
    "#8c564b", // Brown
    "#e377c2", // Pink
    "#7f7f7f", // Gray
    "#bcbd22", // Lime
    "#17becf"  // Cyan
};

Polygon_2 polygonROS2CGAL(geometry_msgs::msg::Polygon &msg);
Direction_2 orientation2dir(geometry_msgs::msg::Quaternion &orientation);
double dir2ang(Direction_2 dir);
Polygon_2 circle2poly(Circle_2 &c, int n);
void draw_circle(const Point_2 &center, const double r, std::string color, bool fill = false, int resolution = 90);
void draw_poly(const Polygon_2 &p, std::string color, bool fill = false);
bool point_in_bbox(const Point_2 &p, const Bbox_2 &box);
bool segment_polygon_intersection(const Segment_2 &s, const Polygon_2 &p);
void draw_segment(const Segment_2 &s, std::string color, double linewidth = 1);
void draw_polyline(const std::vector<Point_2> &points, std::string color, double linewidth = 1);
void draw_points(const std::vector<Point_2> &points, std::string color);
void draw_arrows(const std::vector<Point_2> &points, const std::vector<double> &angles, std::string color);


void plt_show();

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
    bool segmentCollides(const Segment_2 &s) const;
    void offset(double r);
};

class Arc_2
{
private:
    Circle_2 circle;
    Point_2 p0, p1;
    double th0, th1, k;
    int sign;

public:
    Arc_2() {};
    Arc_2(const Point_2 &center, const double &_k, const double &_th0, const double &_th1, const int &_sign)
    {
        sign = _sign;
        k = _k;
        circle = Circle_2(center, 1. / (k * k));
        th0 = _th0;
        th1 = _th1;
        p0 = Point_2(center.x() + sign / k * std::sin(th0),
                     center.y() - sign / k * std::cos(th0));
        p1 = Point_2(center.x() + sign / k * std::sin(th1),
                     center.y() - sign / k * std::cos(th1));
    }
    Point_2 source() const { return p0; }
    Point_2 target() const { return p1; }
    Point_2 eval(double t) const { return Point_2(circle.center().x() + sign / k * std::sin(th0 + t * (th1 - th0)),
                                            circle.center().y() - sign / k * std::cos(th0 + t * (th1 - th0))); }
    Circle_2 getCircle() const { return circle; }
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
    void offsetAllPolys();
    void setVictims(std::vector<Weighted_point_2> &v) { victims = v; };
    void setShelfinoInitPose(Ray_2 &pose) { shelfino = pose; };
    void setGatePose(Ray_2 &pose) { gate = pose; };
    void setShelfinoRadius(double r) { shelfino_r = r; }
    void display();
    Bbox_2 getBbox();
    bool isFree(const Point_2 &p) const;
    bool isFree(const Segment_2 &s) const;
    bool isFree(const Arc_2 &s, const int n_samples) const;
    bool isPOI(const Point_2 &p);
    Ray_2 getShelfino() { return shelfino; }
    Ray_2 getGate() { return gate; }
    std::vector<Weighted_point_2> getVictims() { return victims; }
    std::vector<Point_2> getPOIs()
    {
        std::vector<Point_2> vec;
        vec.push_back(shelfino.source());
        for (auto v : victims)
            vec.push_back(v.point());
        vec.push_back(gate.source());
        return vec;
    }
};
