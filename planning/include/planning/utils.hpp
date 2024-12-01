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

double parseShelfinoRadius(std::string &urdfString) {
    auto parser = urdf::Model{};
    parser.initString(urdfString);
    auto link = parser.getLink("shelfino1/chassis_link");
    auto box = std::dynamic_pointer_cast<urdf::Box>(link->visual->geometry);
    return std::sqrt(std::pow(box->dim.x, 2) + std::pow(box->dim.y, 2));
}

std::pair<std::vector<double>, std::vector<double>> points2coords(const std::vector<Point_2> &points) {
    std::vector<double> xs, ys;
    for (auto p : points) {
        xs.push_back(p.x());
        ys.push_back(p.y());
    }
    return std::make_pair(xs, ys);
}