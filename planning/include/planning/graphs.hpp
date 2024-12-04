#pragma once
#include <vector>
#include <memory>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Point_set_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_data_structure_2.h>


// typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;

class Edge
{
private:
    double length;

public:
    Edge(double l) : length{l} {};
    double getLength() { return length; }
    virtual Segment_2 getSegment() {}
};

class SegmentEdge : public Edge
{
private:
    Segment_2 seg;

public:
    SegmentEdge(Segment_2 s) : Edge(std::sqrt(s.squared_length())) { seg = s; };
    Segment_2 getSegment() { return seg; }
};

class Vertex : public Point_2
{
private:
    std::shared_ptr<std::vector<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Edge>>>> neighbours;
    int index;

public:
    Vertex(double x, double y) : Point_2(x, y)
    {
        neighbours = std::make_shared<std::vector<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Edge>>>>();
    };
    Vertex(Point_2 p) : Point_2(p) {
        neighbours = std::make_shared<std::vector<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Edge>>>>();
    }
    std::shared_ptr<std::vector<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Edge>>>> getNeighbours()
    {
        return neighbours;
    }
    void connect(std::shared_ptr<Vertex> v, std::shared_ptr<Edge> p);
    void setIndex(int i) { index = i; }
    int getIndex() { return index; }
};

typedef CGAL::Triangulation_vertex_base_with_info_2<std::shared_ptr<Vertex>, K> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Point_set_2<K, Tds> Pset;
typedef Pset::Vertex_handle Vertex_handle;

class Graph
{
private:
    std::shared_ptr<std::vector<std::shared_ptr<Edge>>> E;
    std::shared_ptr<std::vector<std::shared_ptr<Vertex>>> V;
    Pset knn_cache;
    bool knn_cache_built = false;
    int currentVertexIndex = 0;

public:
    Graph()
    {
        V = std::make_shared<std::vector<std::shared_ptr<Vertex>>>();
        E = std::make_shared<std::vector<std::shared_ptr<Edge>>>();
    };
    void addVertex(std::shared_ptr<Vertex> v)
    {
        v->setIndex(V->size());
        V->push_back(v);
    }
    std::shared_ptr<std::vector<std::shared_ptr<Vertex>>> getVerteces() { return V; }
    std::shared_ptr<std::vector<std::shared_ptr<Edge>>> getEdges() { return E; }
    std::shared_ptr<std::vector<std::shared_ptr<Vertex>>> KNN(std::shared_ptr<Vertex> q, int k);
    void connect(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2, std::shared_ptr<Edge> p);
    std::vector<std::shared_ptr<Vertex>> dijkstra(std::shared_ptr<Vertex> v0, std::shared_ptr<Vertex> v1);
    size_t size() { return V->size(); }
};
