#include "mapgeometry.hpp"

void plt_show() { plt::show(); }

void draw_segment(const Segment_2 &s, std::string color, double linewidth)
{
    plt::plot({s[0].x(), s[1].x()}, {s[0].y(), s[1].y()}, {{"color", color}, {"linewidth", std::to_string(linewidth)}});
}

void draw_polyline(const std::vector<Point_2> &points, std::string color, double linewidth)
{
    std::vector<double> xs, ys;
    for (auto &p : points)
    {
        xs.push_back(p.x());
        ys.push_back(p.y());
    }
    plt::plot(xs, ys, {{"color", color}, {"linewidth", std::to_string(linewidth)}});
}

void draw_points(const std::vector<Point_2> &points, std::string color)
{
    std::vector<double> xs, ys;
    for (auto &p : points)
    {
        xs.push_back(p.x());
        ys.push_back(p.y());
    }
    plt::scatter(xs, ys, 1, {{"color", color}});
}

void draw_arrows(const std::vector<Point_2> &points, const std::vector<double> &angles, std::string color)
{
    std::vector<double> xs, ys;
    for (size_t i = 0; i < points.size(); i++)
    {
        double x = points.at(i).x();
        double y = points.at(i).y();
        double dx = std::cos(angles.at(i));
        double dy = std::sin(angles.at(i));
        printf("%f, %f, %f, %f\n", x, y, dx, dy);
        plt::arrow(x, y, dx, dy);
        // plt::arrow(1, 1, 1, 1);
    }
    
    plt::set_aspect_equal();//REMOVE
}

Direction_2 orientation2dir(geometry_msgs::msg::Quaternion &orientation)
{
    double qx = orientation.x;
    double qy = orientation.y;
    double qz = orientation.z;
    double qw = orientation.w;

    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    double dx = std::cos(yaw);
    double dy = std::sin(yaw);

    return Direction_2(dx, dy);
}

bool Obstacle::contains(const Point_2 &p)
{
    if (t == CIRCLE)
    {
        return circle.has_on_bounded_side(p);
    }
    else
    {
        return polygon.has_on_bounded_side(p);
    }
}

void Map::display()
{
    draw_poly(border, "k");
    for (auto o : obstacles)
    {
        draw_poly(o.getPoly(), "m", true);
    }
    for (auto v : victims)
    {
        draw_circle(v.point(), 0.5, "y", true);
        plt::text(v.point().x(), v.point().y(), std::to_string(int(v.weight())));
    }
    auto src = gate.source();
    auto diff = gate.to_vector();
    auto perp = diff.perpendicular(CGAL::POSITIVE);
    plt::plot({(src + perp).x(), (src - perp).x()},
              {(src + perp).y(), (src - perp).y()},
              {{"color", "r"}});
    plt::arrow(src.x(), src.y(), diff.x(), diff.y());

    src = shelfino.source();
    diff = shelfino.to_vector();
    draw_circle(src, shelfino_r, "g", true);
    plt::arrow(src.x(), src.y(), diff.x(), diff.y());

    plt::set_aspect_equal();
}

void draw_circle(const Point_2 &center, const double r, std::string color, bool fill, int resolution)
{
    Circle_2 c{center, std::pow(r, 2)};
    Polygon_2 p = circle2poly(c, resolution);
    draw_poly(p, color, fill);
}

void draw_poly(const Polygon_2 &p, std::string color, bool fill)
{
    size_t N = p.size();
    std::vector<double> x(N + 1), y(N + 1);
    if (p.size() == 0)
    {
        std::cerr << "Error: Polygon is empty." << std::endl;
        return;
    }
    for (size_t i = 0; i < N + 1; i++)
    {
        x.at(i) = p[i % N].x();
        y.at(i) = p[i % N].y();
    }
    if (fill)
    {
        plt::fill(x, y, {{"color", color}});
    }
    else
    {
        plt::plot(x, y, {{"color", color}});
    }
}

Polygon_2 polygonROS2CGAL(geometry_msgs::msg::Polygon &msg)
{
    std::vector<Point_2> v;
    for (auto rosPoint : msg.points)
    {
        v.emplace_back(rosPoint.x, rosPoint.y);
    }
    return Polygon_2(v.begin(), v.end());
}

Obstacle::Obstacle(obstacles_msgs::msg::ObstacleMsg &o)
{
    if (o.radius == 0)
    {
        t = POLYGON;
        polygon = polygonROS2CGAL(o.polygon);
    }
    else
    {
        t = CIRCLE;
        auto center = o.polygon.points.at(0); // ASSUMPTION
        circle = Circle_2(Point_2(center.x, center.y), std::pow(o.radius, 2));
        radius = o.radius;
    }
}

Polygon_2 circle2poly(Circle_2 &c, int n)
{
    std::vector<Point_2> v;
    for (int i = 0; i < n; i++)
    {
        double x = c.center().x() + std::sqrt(c.squared_radius()) * std::cos(i * 2 * M_PI / n);
        double y = c.center().y() + std::sqrt(c.squared_radius()) * std::sin(i * 2 * M_PI / n);
        v.emplace_back(x, y);
    }
    return Polygon_2(v.begin(), v.end());
}

Bbox_2 Map::getBbox()
{
    return border.bbox();
}

bool point_in_bbox(const Point_2 &p, const Bbox_2 &box)
{
    return (p.x() >= box.xmin() && p.x() <= box.xmax() &&
            p.y() >= box.ymin() && p.y() <= box.ymax());
}

bool Map::isFree(const Point_2 &p)
{
    if (border.has_on_unbounded_side(p))
        return false;
    for (auto o : obstacles)
    {
        if (o.contains(p))
            return false;
    }
    return true;
}

bool segment_polygon_intersection(const Segment_2 &s, const Polygon_2 &p)
{
    auto it = p.edges_begin();
    while (it != p.edges_end())
    {
        if (CGAL::do_intersect(*it, s))
            return true;
        it++;
    }
    return false;
}

bool Obstacle::segmentCollides(const Segment_2 &s)
{
    if (t == CIRCLE)
    {
        return CGAL::do_intersect(circle, s);
    }
    else
    {
        return segment_polygon_intersection(s, polygon);
    }
}

bool Map::isFree(const Segment_2 &s)
{
    if (segment_polygon_intersection(s, border))
        return false;

    for (auto &obst : obstacles)
    {
        if (obst.segmentCollides(s))
            return false;
    }
    return true;
}

bool Map::isPOI(const Point_2 &p)
{
    for (auto v : victims)
    {
        if (v.point() == p)
            return true;
    }
    if (shelfino.source() == p || gate.source() == p)
        return true;
    return false;
}

void Obstacle::offset(double r)
{
    if (t == CIRCLE)
    {
        circle = Circle_2(circle.center(), std::pow(radius + r, 2));
    }
    else
    {
        auto polyVec = CGAL::create_exterior_skeleton_and_offset_polygons_2(r, polygon);
        polygon = *polyVec.at(0);
    }
}

void Map::offsetAllPolys()
{
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        obstacles[i].offset(shelfino_r);
    }
    auto polyVec = CGAL::create_interior_skeleton_and_offset_polygons_2(shelfino_r, border);
    border = *polyVec.at(0);
}
