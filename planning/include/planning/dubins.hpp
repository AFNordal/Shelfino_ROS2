#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "dubins_solvers.hpp"
#include "mapgeometry.hpp"

#define ARC_COLLISION_SAMPLES 10

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Circle_2 Circle_2;
typedef K::Segment_2 Segment_2;

typedef std::pair<std::array<double, 3>, std::array<int, 3>> DubinsParams;

typedef struct
{
    double th0, th1;
    double k;
    double scale;
} StdDubinsProblem;

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
    std::vector<Point_2> getPointApprox(double spacing);
    DubinsPathSegmentType getType() { return t; }
    Point_2 getEndPoint();
    Arc_2 getArc() { return arc; }
    Segment_2 getSegment() { return seg; }
    bool isFreeIn(const Map &map, const int arc_samples);
};

class SPDubinsPath
{
private:
    std::vector<DubinsPathSegment> segments;
    Point_2 p0, p1;
    double th0, th1;
    double k; // curvature

public:
    SPDubinsPath() {}
    SPDubinsPath(Point_2 _p0, Point_2 _p1, double _th0, double _th1, double _k, DubinsParams p)
    {
        p0 = _p0;
        p1 = _p1;
        th0 = _th0;
        th1 = _th1;
        k = _k;
        setFromParams(p);
    }

    void setFromParams(DubinsParams p);
    std::vector<Point_2> getPolyline(int res);
    std::vector<Point_2> getPointApprox(double spacing);
    Point_2 getEndPoint();
    std::vector<DubinsPathSegment> getSegments() { return segments; }
    bool isFreeIn(const Map &map)
    {
        for (auto &s : segments)
        {
            if (!s.isFreeIn(map, ARC_COLLISION_SAMPLES))
                return false;
        }
        return true;
    }
};

StdDubinsProblem createStdDubinsProblem(Point_2 p0, Point_2 p1, double th0, double th1, double k);
std::pair<double, SPDubinsPath> optimalDubinsParams(Point_2 p0, Point_2 p1, double th0, double th1, double k, const Map &map, const double obstr_cost=1000);
std::pair<double, int> optimalMPDubinsParams(std::vector<SPDubinsPath> &sol_paths,
                             const std::vector<Point_2> &ps,
                             double th0, double th1, double k, const size_t &angRes,
                             bool th0_constrained, bool th1_constrained,
                             const Map &map, const double obstr_cost=1000);

class MPDubinsPath
{
private:
public:
};