#pragma once
#include <stdlib.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Rotational_sweep_visibility_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/intersections.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/intersections.h>

#include "matplotlibcpp/matplotlibcpp.h"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"

// typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel ExactK;
typedef K::Point_2 Point_2;
typedef K::Circle_2 Circle_2;
typedef K::Weighted_point_2 Weighted_point_2;
typedef K::Segment_2 Segment_2;
typedef ExactK::Segment_2 Exact_segment_2;
typedef K::Ray_2 Ray_2;
typedef K::Direction_2 Direction_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<ExactK> Exact_polygon_with_holes_2;
typedef CGAL::Polygon_2<ExactK> Exact_polygon_2;
typedef CGAL::Bbox_2 Bbox_2;

typedef CGAL::Arr_segment_traits_2<K> Traits_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef Arrangement_2::Face_const_handle Face_const_handle;
typedef CGAL::Rotational_sweep_visibility_2<Arrangement_2, CGAL::Tag_false> Visibility_2;
typedef Arrangement_2::Edge_const_iterator Edge_const_iterator;

inline CGAL::Cartesian_converter<K, ExactK> exactifier;
inline CGAL::Cartesian_converter<ExactK, K> inexactifier;

namespace plt = matplotlibcpp;

constexpr std::array<const char *, 10> matplotlib_color_range = {
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
bool polygon_polygon_intersection(const Polygon_2 &p1, const Polygon_2 &p2);
bool polygon_polygon_edge_intersection(const Polygon_2 &p1, const Polygon_2 &p2);
void draw_segment(const Segment_2 &s, std::string color, double linewidth = 1);
void draw_polyline(const std::vector<Point_2> &points, std::string color, double linewidth = 1);
void draw_points(const std::vector<Point_2> &points, std::string color, double s = 1);
void draw_arrows(const std::vector<Point_2> &points, const std::vector<double> &angles, std::string color);
Polygon_2 offsetPolygon(Polygon_2 p, double r);

Exact_polygon_2 inexact2exact(Polygon_2 p);
Exact_segment_2 inexact2exact(Segment_2 p);
Polygon_2 exact2inexact(Exact_polygon_2 p);

void plt_show(double pause_t=10000);
void plt_draw();
void plt_clear();

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
    Obstacle() {}
    Obstacle(obstacles_msgs::msg::ObstacleMsg &o);
    Obstacle(Polygon_2 &p) { polygon = p; t = POLYGON; }
    Obstacle(Circle_2 &c) { circle = c; radius = std::sqrt(c.squared_radius()); t = CIRCLE; }
    Polygon_2 getPoly(const int c_res) { return (t == CIRCLE) ? circle2poly(circle, c_res) : polygon; }
    bool contains(const Point_2 &p);
    bool segmentCollides(const Segment_2 &s) const;
    void offset(double r);
    Obstacle calculateOffset(double r);
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
    Point_2 gateProjection;
    double shelfino_r;
    Arrangement_2 visibilityCache;
    Face_const_handle main_face;
    bool visibilityCacheComputed = false;
    bool pathSet = false;
    std::vector<Point_2> path;

public:
    Map() {};
    void setBorder(geometry_msgs::msg::Polygon &p)
    {
        border = polygonROS2CGAL(p);
    }
    void setObstacles(std::vector<Obstacle> &o);
    std::vector<Obstacle> getObstacles() { return obstacles; }
    void offsetAllPolys();
    void setVictims(std::vector<Weighted_point_2> &v) { victims = v; };
    void setShelfinoPose(Ray_2 &pose) { shelfino = pose; };
    void setGatePose(Ray_2 &pose) { gate = pose; gateProjection = pose.source(); };
    void setShelfinoRadius(double r) { shelfino_r = r; }
    void setPath(std::vector<Point_2> &p) { path = p; pathSet = true; }
    void display();
    void draw_border();
    void draw_obstacles();
    void draw_victims();
    void draw_path();
    void draw_gate();
    void draw_shelfino();
    Bbox_2 getBbox();
    bool isFree(const Point_2 &p) const;
    bool isFree(const Segment_2 &s) const;
    bool isFree(const Arc_2 &s, const int n_samples) const;
    bool isWithinBorder(const Point_2 &p) const;
    bool isPOI(const Point_2 &p);
    Ray_2 getShelfino() { return shelfino; }
    Ray_2 getGate() { return gate; }
    Point_2 getGateProjection() { return gateProjection; }
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
    std::vector<Point_2> getReflexVertices(std::vector<Point_2> &offsetRVs, std::vector<double> &inboundAngs, std::vector<double> &outboundAngs);
    std::vector<Point_2> visibilityQuery(Point_2 queryPoint);
};
