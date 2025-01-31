#include "graphs.hpp"

// Find K+1 nearest points (cartesian distance) in graph using CGAL. K+1 since it also finds itself
std::shared_ptr<std::vector<std::shared_ptr<Vertex>>> Graph::KNN(std::shared_ptr<Vertex> q, int k)
{
    // Create KNN cache first time
    if (!knn_cache_built)
    {
        std::vector<std::pair<Point_2, std::shared_ptr<Vertex>>> vec;
        for (auto &v : *V)
            vec.push_back(std::make_pair(*v, v));
        knn_cache.insert(vec.begin(), vec.end());
        knn_cache_built = true;
    }
    // Make actual query
    std::list<Vertex_handle> _res;
    knn_cache.nearest_neighbors(*q, k + 1, std::back_inserter(_res));
    auto res = std::make_shared<std::vector<std::shared_ptr<Vertex>>>();
    for (auto &vh : _res)
        res->push_back(vh->info());
    return res;
}

void Vertex::connect(std::shared_ptr<Vertex> v, std::shared_ptr<Edge> p)
{
    neighbours->push_back(std::make_pair(v, p));
}

void Graph::connect(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2, std::shared_ptr<Edge> p)
{
    v1->connect(v2, p);
    v2->connect(v1, p);
    E->push_back(p);
}

// For priority queue for dijkstra
typedef std::pair<double, std::shared_ptr<Vertex>> DVPair;
class DVPairComparator
{
public:
    bool operator()(DVPair p1, DVPair p2)
    {
        return (p1.first > p2.first);
    }
};

// Priority queue based dijkstra-shortest-path
std::vector<std::shared_ptr<Vertex>> Graph::dijkstra(std::shared_ptr<Vertex> v0, std::shared_ptr<Vertex> v1)
{
    std::priority_queue<DVPair, std::vector<DVPair>, DVPairComparator> pq;
    std::vector<double> dist(size(), INFINITY);
    std::vector<int> path_tracker(size(), 0);
    pq.push(std::make_pair(0, v0));
    dist[v0->getIndex()] = 0;

    while (!pq.empty())
    {
        std::shared_ptr<Vertex> q = pq.top().second;
        pq.pop();
        for (auto connection : *(q->getNeighbours()))
        {
            std::shared_ptr<Vertex> n = connection.first;
            double len = connection.second->getLength();
            if (dist.at(n->getIndex()) > dist.at(q->getIndex()) + len)
            {
                dist.at(n->getIndex()) = dist.at(q->getIndex()) + len;
                pq.push(std::make_pair(dist.at(n->getIndex()), n));
                path_tracker.at(n->getIndex()) = q->getIndex();
            }
        }
    }
    std::vector<std::shared_ptr<Vertex>> sol;
    sol.push_back(v1);
    int current_vertex_idx = v1->getIndex();
    while (current_vertex_idx != v0->getIndex())
    {
        current_vertex_idx = path_tracker.at(current_vertex_idx);
        sol.push_back(V->at(current_vertex_idx));
    }
    return sol;
}
