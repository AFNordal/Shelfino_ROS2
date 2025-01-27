#include "dubins.hpp"

double arr_sum(std::array<double, 3> s)
{
    return s.at(0) + s.at(1) + s.at(2);
}

std::array<double, 3> arr_mul(const std::array<double, 3> &s, const double &x)
{
    return std::array<double, 3>{s.at(0) * x, s.at(1) * x, s.at(2) * x};
}

bool DubinsPathSegment::isFreeIn(const Map &map, const int arc_samples)
{
    if (t == ARC)
        return map.isFree(arc, arc_samples);
    else
        return map.isFree(seg);
}

// void SPDubinsPath::createOptimal()
// {
//     auto optimalParams = optimalDubinsParams(p0, p1, th0, th1, k);
//     setFromParams(optimalParams);
// }

void SPDubinsPath::setFromParams(DubinsParams p)
{
    std::array<double, 3> lengths = p.first;
    std::array<int, 3> signs = p.second;

    segments.emplace_back(p0, th0, lengths[0], signs[0], k);
    segments.emplace_back(segments.back().getP1(),
                          segments.back().getTh1(),
                          lengths[1], signs[1], k);
    segments.emplace_back(segments.back().getP1(),
                          segments.back().getTh1(),
                          lengths[2], signs[2], k);
}

std::pair<double, SPDubinsPath> optimalDubinsParams(Point_2 p0, Point_2 p1, double th0, double th1, double k, const Map &map, const double obstr_cost)
{
    double record_l = INFINITY;
    SPDubinsPath best_path;
    StdDubinsProblem prob = createStdDubinsProblem(p0, p1, th0, th1, k);
    for (size_t i = 0; i < solvers.size(); i++)
    {
        SolType _s = solvers.at(i)(prob.th0, prob.th1, prob.k);
        bool ok = _s.first;
        if (!ok)
            continue;
        auto lengths = arr_mul(_s.second, prob.scale);
        DubinsParams params = std::make_pair(lengths, sign_configs.at(i));
        SPDubinsPath p{p0, p1, th0, th1, k, params};
        double l = lengths.at(0) + lengths.at(1) + lengths.at(2);
        if (l < record_l && (!p.isFreeIn(map)))
            l += obstr_cost;
        if (l < record_l)
        {
            record_l = l;
            best_path = p;
        }
    }
    if (record_l == INFINITY)
        printf("[WARNING]: No feasible SP dubins found\n");
    return std::make_pair(record_l, best_path);
}

StdDubinsProblem createStdDubinsProblem(Point_2 p0, Point_2 p1, double th0, double th1, double k)
{
    StdDubinsProblem prob;
    double dx = p1.x() - p0.x();
    double dy = p1.y() - p0.y();
    prob.scale = std::sqrt(dx * dx + dy * dy) / 2;
    double phi = std::atan2(dy, dx);
    prob.k = prob.scale * k;

    prob.th0 = mod2pi(th0 - phi);
    prob.th1 = mod2pi(th1 - phi);
    return prob;
}

std::vector<Point_2> DubinsPathSegment::getPolyline(int res)
{
    std::vector<Point_2> line;
    if (t == SEGMENT)
    {
        line.push_back(seg[0]);
        line.push_back(seg[1]);
    }
    else
    {
        int n_points = std::max(int(fabs(th1 - th0) / (2 * M_PI) * res), 2);
        for (int i = 0; i < n_points; i++)
        {
            line.push_back(arc.eval(i * 1. / (n_points - 1)));
        }
    }
    return line;
}

std::vector<Point_2> SPDubinsPath::getPolyline(int res)
{
    std::vector<Point_2> line;
    for (auto &seg : segments)
    {
        std::vector<Point_2> l = seg.getPolyline(res);
        line.insert(line.end(), l.begin(), l.end());
    }
    return line;
}

std::vector<Point_2> DubinsPathSegment::getPointApprox(double spacing)
{
    std::vector<Point_2> points;
    size_t n_points = int(length / spacing);
    if (t == SEGMENT)
    {
        for (int i = 0; i < n_points; i++)
        {
            double _t = double(i) / n_points;
            Point_2 p = seg.source() + seg.to_vector() * _t;
            points.push_back(p);
        }
    }
    else
    {
        for (int i = 0; i < n_points; i++)
        {
            points.push_back(arc.eval(i * 1. / n_points));
        }
    }
    return points;
}

std::vector<Point_2> SPDubinsPath::getPointApprox(double spacing)
{
    std::vector<Point_2> points;
    for (auto &seg : segments)
    {
        std::vector<Point_2> l = seg.getPointApprox(spacing);
        points.insert(points.end(), l.begin(), l.end());
    }
    return points;
}

Point_2 DubinsPathSegment::getEndPoint()
{
    if (t == SEGMENT)
    {
        return seg.target();
    }
    else
    {
        return arc.target();
    }
}

Point_2 SPDubinsPath::getEndPoint()
{
    return segments.back().getEndPoint();
}

// Returns path length and # of collisions
std::pair<double, int> optimalMPDubinsParams(std::vector<SPDubinsPath> &sol_paths,
                                             const std::vector<Point_2> &ps,
                                             double th0, double th1, double k, const size_t &angRes,
                                             bool th0_constrained, bool th1_constrained,
                                             const Map &map, std::vector<int> &collisions, const double obstr_cost)
{
    const size_t N = ps.size();
    assert(N >= 2);
    collisions = std::vector<int>(N - 1, 0);
    if (N == 2 && th1_constrained && th0_constrained)
    {
        auto res = optimalDubinsParams(ps.at(0), ps.at(1), th0, th1, k, map, obstr_cost);
        sol_paths = std::vector<SPDubinsPath>{};
        sol_paths.push_back(res.second);
        int n_col = int(res.first / obstr_cost);
        collisions.at(0) = n_col;
        return std::make_pair(fmod(res.first, obstr_cost), n_col);
    }
    int it_hi = th1_constrained ? N - 3 : N - 2;
    int it_lo = th0_constrained ? 1 : 0;
    std::vector<std::vector<double>> L(N - 1, std::vector<double>(angRes, 0));
    std::vector<std::vector<size_t>> ang_idx_tracker(it_hi + 1, std::vector<size_t>(angRes, 0));
    std::vector<std::vector<std::pair<double, SPDubinsPath>>> sol_tracker(N - 1, std::vector<std::pair<double, SPDubinsPath>>(angRes, std::make_pair(0., SPDubinsPath())));
    if (th1_constrained)
    {
        for (size_t h = 0; h < angRes; h++)
        {
            double ang = h * 2 * M_PI / angRes;
            auto res = optimalDubinsParams(ps.at(N - 2), ps.at(N - 1), ang, th1, k, map, obstr_cost);
            L.at(N - 2).at(h) = res.first;
            sol_tracker.at(N - 2).at(h) = res;
        }
    }
    for (int i = it_hi; i >= it_lo; i--)
    {
        for (size_t h1 = 0; h1 < angRes; h1++)
        {
            double ang1 = h1 * 2 * M_PI / angRes;
            double shortest = INFINITY;
            size_t best_h2 = 0;
            for (size_t h2 = 0; h2 < angRes; h2++)
            {
                double ang2 = h2 * 2 * M_PI / angRes;
                auto res = optimalDubinsParams(ps.at(i), ps.at(i + 1), ang1, ang2, k, map, obstr_cost);
                double l;
                if (i == N - 2)
                    l = res.first;
                else
                    l = res.first + L.at(i + 1).at(h2);
                if (l < shortest)
                {
                    shortest = l;
                    best_h2 = h2;
                    sol_tracker.at(i).at(h1) = res;
                }
            }
            ang_idx_tracker.at(i).at(h1) = best_h2;
            L.at(i).at(h1) = shortest;
        }
    }
    double shortest = INFINITY;
    size_t best_h = 0;
    if (th0_constrained)
    {
        for (size_t h = 0; h < angRes; h++)
        {
            double ang = h * 2 * M_PI / angRes;
            auto res = optimalDubinsParams(ps.at(0), ps.at(1), th0, ang, k, map, obstr_cost);
            double l;
            if (N > 2)
                l = res.first + L.at(1).at(h);
            else
                l = res.first;
            if (l < shortest)
            {
                shortest = l;
                best_h = h;
                sol_tracker.at(0).at(h) = res;
            }
        }
    }
    sol_paths = std::vector<SPDubinsPath>(N - 1, SPDubinsPath());
    double total_length;
    if (th0_constrained)
    {
        sol_paths.at(0) = sol_tracker.at(0).at(best_h).second;
        int n_col = int(sol_tracker.at(0).at(best_h).first / obstr_cost);
        collisions.at(0) = n_col;
    }
    else
    {
        for (size_t h = 0; h < angRes; h++)
        {
            if (L.at(0).at(h) < shortest)
            {
                shortest = L.at(0).at(h);
                best_h = h;
            }
        }
    }
    total_length = shortest;
    for (size_t i = it_lo; i < it_hi + 2; i++)
    {
        if (i < N - 1)
        {
            sol_paths.at(i) = sol_tracker.at(i).at(best_h).second;
            int n_col = int(sol_tracker.at(i).at(best_h).first / obstr_cost);
            collisions.at(i) = n_col;
        }
        if (i < N - 2)
            best_h = ang_idx_tracker.at(i).at(best_h);
    }
    return std::make_pair(fmod(total_length, obstr_cost), int(total_length / obstr_cost));
}