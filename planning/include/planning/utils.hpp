#include <urdf/model.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <utility>
#include <vector>

#include <CGAL/Simple_cartesian.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_2;

double parseShelfinoRadius(const std::string &urdfString)
{
    auto parser = urdf::Model{};
    if (!parser.initString(urdfString))
    {
        printf("BAD INIT");
        return 1;
    }
    auto link = parser.getLink("shelfino1/chassis_link");
    if (!link) {
        printf("NO LINK");
        return 1;
    }
    if (!(link->visual)) {
        printf("NO VISUAL");
        return 1;
    }
    auto box = std::dynamic_pointer_cast<urdf::Box>(link->visual->geometry);
    if (!box) {
        printf("NO BOX");
        return 1;
    }
    return std::sqrt(std::pow(box->dim.x, 2) + std::pow(box->dim.y, 2));
}

std::pair<std::vector<double>, std::vector<double>> points2coords(const std::vector<Point_2> &points)
{
    std::vector<double> xs, ys;
    for (auto p : points)
    {
        xs.push_back(p.x());
        ys.push_back(p.y());
    }
    return std::make_pair(xs, ys);
}