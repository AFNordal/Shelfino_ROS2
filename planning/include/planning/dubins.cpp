#include "dubins.hpp"

void SPDubinsPath::createOptimal()
{

    StdDubinsProblem stdProb = createStdDubinsProblem(p0, p1, th0, th1, k);
    auto optimalParams = optimalDubinsParams(stdProb);
    std::array<double, 3> lengths = optimalParams.first;
    std::array<int, 3> signs = optimalParams.second;

    segments.emplace_back(p0, th0, lengths[0] * stdProb.scale, signs[0], k);
    segments.emplace_back(segments.back().getP1(),
                          segments.back().getTh1(),
                          lengths[1] * stdProb.scale, signs[1], k);
    segments.emplace_back(segments.back().getP1(),
                          segments.back().getTh1(),
                          lengths[2] * stdProb.scale, signs[2], k);
}

std::pair<std::array<double, 3>, std::array<int, 3>> optimalDubinsParams(StdDubinsProblem prob)
{
    double record_l = INFINITY;
    std::array<int, 3> best_signs;
    std::array<double, 3> best_lengths;
    for (int i = 0; i < solvers.size(); i++)
    {
        SolType _s = solvers.at(i)(prob.th0, prob.th1, prob.k);
        bool ok = _s.first;
        if (!ok)
        {
            continue;
        }
        auto lengths = _s.second;
        double l = lengths.at(0) + lengths.at(1) + lengths.at(2);
        if (l < record_l)
        {
            record_l = l;
            best_signs = sign_configs.at(i);
            best_lengths = lengths;
        }
    }
    if (record_l == INFINITY)
        printf("No optimal SP dubins found\n");
    return std::make_pair(best_lengths, best_signs);
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
        int n_points = int(fabs(th1 - th0) / (2 * M_PI) * res);
        for (int i = 0; i < n_points; i++)
        {
            line.push_back(arc.eval(i * 1. / n_points));
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

double arr_sum(std::array<double, 3> s)
{
    return s.at(0) + s.at(1) + s.at(2);
}

void optimalMPDubinsParams(std::vector<Point_2> ps, double th0, double th1, double k, size_t angRes)
{
    // N = len(xs)
    size_t N = ps.size();
    // H = 36
    // L = np.zeros((N - 1, H))
    double L[N - 1][angRes];
    // solutions = np.zeros((N - 1, H, 2, 3))
    // angles = np.zeros((N - 2, H))

    // # j = N - 1
    // for h in range(H):
    for (size_t h = 0; h < angRes; h++)
    {
        //     ang = h * 2 * np.pi / H
        double ang = h * 2 * M_PI / angRes;
        //     lengths, config = sp_dubins(
        //         xs[N - 2], ys[N - 2], xs[N - 1], ys[N - 1], ang, th1, k
        //     )
        StdDubinsProblem stdProb = createStdDubinsProblem(ps[N - 2], ps[N - 1], ang, th1, k);
        auto optimalParams = optimalDubinsParams(stdProb);
        std::array<double, 3> lengths = optimalParams.first;
        std::array<int, 3> signs = optimalParams.second;
        //     L[N - 2, h] = np.sum(lengths)
        L[N - 2][h] = arr_sum(lengths);
        //     solutions[N-2, h, 0, :] = lengths[:]
        //     solutions[N-2, h, 1, :] = config[:]
    }

    // for i in range(N - 3, 0, -1):
    for (size_t i = N - 3; i > 0; i--)
    {
        //     for h1 in range(H):
        for (size_t h1=0; h1<angRes; h1++) {
             // ang1 = h1 * 2 * np.pi / H
            double ang1 = h1 * 2 * M_PI / angRes;
            // shortest = np.inf
            double shortest = INFINITY;
            // for h2 in range(H):
            for (size_t h2=0; h2<angRes; h2++) {
                // ang2 = h2 * 2 * np.pi / H
                double ang2 = h2 * 2 * M_PI / angRes;
                // lengths, config = sp_dubins(
                //     xs[i], ys[i], xs[i + 1], ys[i + 1], ang1, ang2, k
                // )
                StdDubinsProblem stdProb = createStdDubinsProblem(ps[i], ps[i + 1], ang1, ang2, k);
                auto optimalParams = optimalDubinsParams(stdProb);
                std::array<double, 3> lengths = optimalParams.first;
                std::array<int, 3> signs = optimalParams.second;
                // l = np.sum(lengths) + L[i + 1, h2]
                double l = arr_sum(lengths) + L[i+1][h2];
                // if l < shortest:
                if (l<shortest) {
                    // shortest = l
                    shortest = l;
                    // best_h2 = h2
                    // best_sol = np.array((lengths, config))
                }
            }
        // angles[i, h1] = best_h2
        // L[i, h1] = shortest
            L[i][h1] = shortest;
        // solutions[i, h1, :, :] = best_sol[:, :]
        }
    }

    // # j = 1
    // shortest = np.inf
    double shortest = INFINITY;
    // for h in range(H):
    for (size_t h = 0; h < angRes; h++) {
    //     ang = h * 2 * np.pi / H
        double ang = h * 2 * M_PI / angRes;
    //     lengths, config = sp_dubins(xs[0], ys[0], xs[1], ys[1], th0, ang, k)
        StdDubinsProblem stdProb = createStdDubinsProblem(ps[0], ps[1], th0, ang, k);
        auto optimalParams = optimalDubinsParams(stdProb);
        std::array<double, 3> lengths = optimalParams.first;
        std::array<int, 3> signs = optimalParams.second;
    //     l = np.sum(lengths) + L[1, h]
        double l = arr_sum(lengths) + L[1][h];
    //     if l < shortest:
        if (l < shortest) {
    //         shortest = l
            shortest = l;
    //         best_h = h
    //         best_sol = np.array((lengths, config))
        }
    }
    // best_angles = np.zeros(N)
    // best_angles[0] = th0
    // best_angles[N - 1] = th1
    // optimal_solution = np.zeros((N-1, 2, 3))
    // optimal_solution[0, :, :] = best_sol[:, :]
    // print(angles)
    // for i in range(1, N - 1):
    //     best_angles[i] = best_h * 2 * np.pi / H
    //     optimal_solution[i, :, :] = solutions[i, best_h, :, :]
    //     if i < N-2:
    //         best_h = int(angles[i, best_h])
    //         print(best_h)

    // return best_angles, optimal_solution[:, 0, :], optimal_solution[:, 1, :]
}