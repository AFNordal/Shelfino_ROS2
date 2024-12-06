#include "dubins_solvers.hpp"


double mod2pi(const double &x)
{
    if (x > 0)
        return fmod(x, 2 * M_PI);
    else
        return 2*M_PI + fmod(x, 2 * M_PI);
}

SolType makeSolType(bool b, double s1, double s2, double s3)
{
    SolType r;
    r.first = b;
    r.second = {s1, s2, s3};
    return r;
}

SolType LSL(double th0, double th1, double k)
{
    double C = std::cos(th1) - std::cos(th0);
    double S = 2 * k + std::sin(th0) - std::sin(th1);
    double r = 1 / k;
    double s1 = r * mod2pi(std::atan2(C, S) - th0);
    double tmp = 2 + 4 * k * k - 2 * std::cos(th0 - th1) + 4 * k * (std::sin(th0) - std::sin(th1));
    if (tmp < 0)
        return makeSolType(false, 0, 0, 0);
    double s2 = r * std::sqrt(tmp);
    double s3 = r * mod2pi(th1 - std::atan2(C, S));
    return makeSolType(true, s1, s2, s3);
}
SolType RSR(double th0, double th1, double k)
{
    double C = std::cos(th0) - std::cos(th1);
    double S = 2 * k - std::sin(th0) + std::sin(th1);
    double r = 1 / k;
    double s1 = r * mod2pi(th0 - std::atan2(C, S));
    double tmp = 2 + 4 * k * k - 2 * std::cos(th0 - th1) - 4 * k * (std::sin(th0) - std::sin(th1));
    if (tmp < 0)
        return makeSolType(false, 0, 0, 0);
    double s2 = r * std::sqrt(tmp);
    double s3 = r * mod2pi(std::atan2(C, S) - th1);
    return makeSolType(true, s1, s2, s3);
}
SolType LSR(double th0, double th1, double k)
{
    double C = std::cos(th0) + std::cos(th1);
    double S = 2 * k + std::sin(th0) + std::sin(th1);
    double r = 1 / k;
    double tmp = -2 + 4 * k * k + 2 * std::cos(th0 - th1) + 4 * k * (std::sin(th0) + std::sin(th1));
    if (tmp < 0)
        return makeSolType(false, 0, 0, 0);
    double s2 = r * std::sqrt(tmp);
    double s1 = r * mod2pi(std::atan2(-C, S) - std::atan2(-2, k * s2) - th0);
    double s3 = r * mod2pi(std::atan2(-C, S) - std::atan2(-2, k * s2) - th1);
    return makeSolType(true, s1, s2, s3);
}
SolType RSL(double th0, double th1, double k)
{
    double C = std::cos(th0) + std::cos(th1);
    double S = 2 * k - std::sin(th0) - std::sin(th1);
    double r = 1 / k;
    double tmp = -2 + 4 * k * k + 2 * std::cos(th0 - th1) - 4 * k * (std::sin(th0) + std::sin(th1));
    if (tmp < 0)
        return makeSolType(false, 0, 0, 0);
    double s2 = r * std::sqrt(tmp);
    double s1 = r * mod2pi(th0 - std::atan2(C, S) + std::atan2(2, k * s2));
    double s3 = r * mod2pi(th1 - std::atan2(C, S) + std::atan2(2, k * s2));
    return makeSolType(true, s1, s2, s3);
}
SolType RLR(double th0, double th1, double k)
{
    double C = std::cos(th0) - std::cos(th1);
    double S = 2 * k - std::sin(th0) + std::sin(th1);
    double r = 1 / k;
    double tmp = (6 - 4 * k * k + 2 * std::cos(th0 - th1) + 4 * k * (std::sin(th0) - std::sin(th1))) / 8;
    if (fabs(tmp) > 1)
        return makeSolType(false, 0, 0, 0);
    double s2 = r * mod2pi(2 * M_PI - std::acos(tmp));
    double s1 = r * mod2pi(th0 - std::atan2(C, S) + k * s2 / 2);
    double s3 = r * mod2pi(th0 - th1 + k * (s2 - s1));
    return makeSolType(true, s1, s2, s3);
}
SolType LRL(double th0, double th1, double k)
{
    double C = std::cos(th1) - std::cos(th0);
    double S = 2 * k + std::sin(th0) - std::sin(th1);
    double r = 1 / k;
    double tmp = (6 - 4 * k * k + 2 * std::cos(th0 - th1) - 4 * k * (std::sin(th0) - std::sin(th1))) / 8;
    if (fabs(tmp) > 1)
        return makeSolType(false, 0, 0, 0);
    double s2 = r * mod2pi(2 * M_PI - std::acos(tmp));
    double s1 = r * mod2pi(-th0 + std::atan2(C, S) + k * s2 / 2);
    double s3 = r * mod2pi(th1 - th0 + k * (s2 - s1));
    return makeSolType(true, s1, s2, s3);
}

std::array<SolType (*)(double, double, double), 6> solvers = {LSL, RSR, LSR, RSL, RLR, LRL};
std::array<std::array<int, 3>, 6> sign_configs = {{{1, 0, 1},
                                                   {-1, 0, -1},
                                                   {1, 0, -1},
                                                   {-1, 0, 1},
                                                   {-1, 1, -1},
                                                   {1, -1, 1}}};