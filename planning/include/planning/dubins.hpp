#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "dubins_solvers.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Circle_2 Circle_2;
typedef K::Segment_2 Segment_2;

typedef struct
{
    double th0, th1;
    double k;
    double scale;
} StdDubinsProblem;

class Arc_2
{
private:
    Circle_2 circle;
    Point_2 p0, p1;
    double th0, th1, k;
    int sign;

public:
    Arc_2() {};
    Arc_2(Point_2 center, double _k, double _th0, double _th1, int _sign)
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
    Point_2 source() { return p0; }
    Point_2 target() { return p1; }
    Point_2 eval(double t) { return Point_2(circle.center().x() + sign / k * std::sin(th0 + t * (th1 - th0)),
                                            circle.center().y() - sign / k * std::cos(th0 + t * (th1 - th0))); }
};

typedef enum
{
    ARC,
    SEGMENT
} DubinsPathSegmentType;

class DubinsPathSegment
{
private:
    Segment_2 seg;
    Arc_2 arc;
    int sign;
    double th0, th1, length;
    DubinsPathSegmentType t;

public:
    DubinsPathSegment(Point_2 p0, double _th0, double _length, int _sign, double k)
    {
        length = _length;
        th0 = _th0;
        sign = _sign;
        if (sign == 0)
        {
            t = SEGMENT;
            th1 = th0;
            Point_2 p1{p0.x() + std::cos(th0) * length, p0.y() + std::sin(th0) * length};
            seg = Segment_2(p0, p1);
        }
        else
        {
            t = ARC;
            Point_2 c{p0.x() - sign / k * std::sin(th0),
                      p0.y() + sign / k * std::cos(th0)};
            th1 = sign * length * k + th0;
            arc = Arc_2(c, k, th0, th1, sign);
        }
    }
    double getTh0() { return th0; }
    double getTh1() { return th1; }
    Point_2 getP0() { return (t == ARC) ? arc.source() : seg[0]; }
    Point_2 getP1() { return (t == ARC) ? arc.target() : seg[1]; }
    std::vector<Point_2> getPolyline(int res);
};

class SPDubinsPath
{
private:
    std::vector<DubinsPathSegment> segments;
    Point_2 p0, p1;
    double th0, th1;
    double k; // curvature

public:
    SPDubinsPath(Point_2 _p0, Point_2 _p1, double _th0, double _th1, double _k)
    {
        p0 = _p0;
        p1 = _p1;
        th0 = _th0;
        th1 = _th1;
        k = _k;
        createOptimal();
    }

    void createOptimal();
    std::vector<Point_2> getPolyline(int res);
};


StdDubinsProblem createStdDubinsProblem(Point_2 p0, Point_2 p1, double th0, double th1, double k);
std::pair<std::array<double, 3>, std::array<int, 3>> optimalDubinsParams(StdDubinsProblem prob);

class MPDubinsPath
{
private:
public:
};