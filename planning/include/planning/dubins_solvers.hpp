#pragma once

#include <tuple>
#include <math.h>

typedef std::pair<bool, std::array<double, 3>> SolType;

double mod2pi(const double &x);

SolType makeSolType(bool b, double s1, double s2, double s3);

SolType LSL(double th0, double th1, double k);
SolType RSR(double th0, double th1, double k);
SolType LSR(double th0, double th1, double k);
SolType RSL(double th0, double th1, double k);
SolType RLR(double th0, double th1, double k);
SolType LRL(double th0, double th1, double k);

extern std::array<SolType (*)(double, double, double), 6> solvers;
extern std::array<std::array<int, 3>, 6> sign_configs;
