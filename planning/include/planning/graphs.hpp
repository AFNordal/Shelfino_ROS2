#pragma once
#include <vector>
#include <memory>
#include <CGAL/Simple_cartesian.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_2;

class Path
{
private:
    double length;
public:
    Path(double l) : length{l} {};
    double getLength() { return length; }
};

class Vertex : public Point_2
{
private:
    std::vector<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Path>>> neighbours;

public:
    Vertex(double x, double y) : Point_2(x, y) {};
};

class Graph
{
private:
    std::vector<std::shared_ptr<Vertex>> V;

public:
    Graph() {};
    void addVertex(std::shared_ptr<Vertex> v) { V.push_back(v); }
};