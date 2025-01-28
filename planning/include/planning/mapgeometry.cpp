#include "mapgeometry.hpp"

void plt_show(double pause_t)
{
    plt::ion();
    plt::show();
    plt::pause(pause_t);
}

void plt_draw() {
    plt::draw();
    plt::pause(0.2);
}

void plt_clear() {
    plt::clf();
}

void draw_segment(const Segment_2 &s, std::string color, double linewidth)
{
    plt::plot({s[0].x(), s[1].x()}, {s[0].y(), s[1].y()}, {{"color", color}, {"linewidth", std::to_string(linewidth)}});
}

double dir2ang(Direction_2 dir)
{
    return std::atan2(dir.dy(), dir.dx());
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

void draw_points(const std::vector<Point_2> &points, std::string color, double s)
{
    std::vector<double> xs, ys;
    for (auto &p : points)
    {
        xs.push_back(p.x());
        ys.push_back(p.y());
    }
    plt::scatter(xs, ys, s, {{"color", color}});
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
        plt::arrow(x, y, dx, dy, color);
    }
}

Direction_2 orientation2dir(geometry_msgs::msg::Quaternion &orientation)
{
    double qx = orientation.x;
    double qy = orientation.y;
    double qz = orientation.z;
    double qw = orientation.w;

    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    return Direction_2(std::cos(yaw), std::sin(yaw));
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
    draw_border();
    draw_obstacles();
    draw_victims();
    draw_gate();
    draw_shelfino();
    draw_path();
}

void Map::draw_path() {
    if (pathSet) {
        draw_polyline(path, "r");
    }
}

void Map::draw_border() {
    draw_poly(border, "k");
    plt::set_aspect_equal();
}

void Map::draw_obstacles() {
    for (auto o : obstacles)
    {
        draw_poly(o.getPoly(10), "m", true);
    }
    plt::set_aspect_equal();
}

void Map::draw_victims() {
    for (auto v : victims)
    {
        draw_circle(v.point(), 0.5, "y", true);
        plt::text(v.point().x(), v.point().y(), std::to_string(int(v.weight())));
    }
    plt::set_aspect_equal();
}

void Map::draw_gate() {
    auto src = gate.source();
    auto diff = gate.to_vector();
    auto perp = diff.perpendicular(CGAL::POSITIVE);
    plt::plot({(src + perp).x(), (src - perp).x()},
              {(src + perp).y(), (src - perp).y()},
              {{"color", "r"}});
    plt::arrow(src.x(), src.y(), diff.x(), diff.y());
    plt::set_aspect_equal();
}

void Map::draw_shelfino() {
    auto src = shelfino.source();
    auto diff = shelfino.to_vector();
    draw_circle(src, shelfino_r, "g", true);
    plt::arrow(src.x(), src.y(), diff.x(), diff.y());
    plt::set_aspect_equal();
}

Exact_polygon_2 inexact2exact(Polygon_2 p)
{
    Exact_polygon_2 cp{};
    for (auto &pt : p)
    {
        cp.push_back(exactifier(pt));
    }
    return cp;
}

Polygon_2 exact2inexact(Exact_polygon_2 cp)
{
    Polygon_2 p{};
    for (auto &pt : cp)
    {
        p.push_back(inexactifier(pt));
    }
    return p;
}

void Map::setObstacles(std::vector<Obstacle> &obst)
{
    for (auto &o : obst)
    {
        // First check of obstacle collides with one previously added
        bool overlaps = false;
        Polygon_2 p = o.getPoly(10);
        for (auto &o_ : obstacles)
        {
            if (polygon_polygon_intersection(p, o_.getPoly(10)))
            {
                overlaps = true;
                break;
            }
        }
        if (!overlaps)
            obstacles.push_back(o);
    }
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
    Polygon_2 p{v.begin(), v.end()};
    if (p.is_clockwise_oriented())
    {
        p.reverse_orientation();
    }
    return p;
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

// circumscribed polygon approximation with n vertices
Polygon_2 circle2poly(Circle_2 &c, int n)
{
    double r = std::sqrt(c.squared_radius());
    double ang = 2 * M_PI / n;
    double outer_r = r / std::cos(ang / 2);

    std::vector<Point_2> v;
    for (int i = 0; i < n; i++)
    {
        double x = c.center().x() + outer_r * std::cos(i * ang);
        double y = c.center().y() + outer_r * std::sin(i * ang);
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

bool Map::isFree(const Point_2 &p) const
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

bool Map::isWithinBorder(const Point_2 &p) const
{
    return border.has_on_bounded_side(p);
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

bool polygon_polygon_intersection(const Polygon_2 &p1, const Polygon_2 &p2)
{
    return CGAL::do_intersect(inexact2exact(p1), inexact2exact(p2));
}

bool polygon_polygon_edge_intersection(const Polygon_2 &p1, const Polygon_2 &p2)
{
    for (auto eit = p1.edges_begin(); eit != p1.edges_end(); eit++)
    {
        if (segment_polygon_intersection(*eit, p2))
            return true;
    }
    return false;
}

bool Obstacle::segmentCollides(const Segment_2 &s) const
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

bool Map::isFree(const Segment_2 &s) const
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

bool Map::isFree(const Arc_2 &a, const int n_samples) const
{
    for (double t = 0; t <= 1; t += 1. / (n_samples - 1))
    {
        if (!isFree(a.eval(t)))
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
        polygon = offsetPolygon(polygon, r);
    }
}

// Obstacle Obstacle::calculateOffset(double r) {
//     Obstacle o;
//     if (t == CIRCLE)
//     {
//         Circle_2 c(circle.center(), std::pow(radius + r, 2));
//         o = Obstacle(c);
//     }
//     else
//     {
//         Polygon_2 p = offsetPolygon(polygon, r);
//         o = Obstacle(p);
//     }
//     return o;
// }

Polygon_2 offsetPolygon(Polygon_2 p, double r)
{
    if (r > 0)
    {
        auto polyVec = CGAL::create_exterior_skeleton_and_offset_polygons_2(r, p);
        Polygon_2 off = *polyVec.at(1);
        if (off.is_clockwise_oriented())
            off.reverse_orientation();
        return off;
    }
    else if (r < 0)
    {
        auto polyVec = CGAL::create_interior_skeleton_and_offset_polygons_2(-r, p);
        Polygon_2 off = *polyVec.at(0);
        if (off.is_clockwise_oriented())
            off.reverse_orientation();
        return off;
    }
    else
        return p;
}

void Map::offsetAllPolys()
{
    for (size_t i = 0; i < obstacles.size(); i++)
        obstacles.at(i).offset(shelfino_r);

    border = offsetPolygon(border, -shelfino_r);

    for (size_t i = 1; i < obstacles.size(); i++)
    {
        Polygon_2 p1 = obstacles.at(i).getPoly(10);
        for (size_t j = 0; j < i; j++)
        {
            Polygon_2 p2 = obstacles.at(j).getPoly(10);
            Exact_polygon_with_holes_2 res;
            if (CGAL::join(inexact2exact(p1), inexact2exact(p2), res))
            {
                printf("[INFO]: Joined overlapping obstacles after offset\n");
                Polygon_2 joined = exact2inexact(res.outer_boundary());
                obstacles.at(i) = Obstacle{joined};
                obstacles.erase(obstacles.begin() + j);
                i--;
                j--;
            }
        }
    }
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        Polygon_2 p = obstacles.at(i).getPoly(10);
        std::list<Exact_polygon_with_holes_2> res;
        if (polygon_polygon_edge_intersection(p, border))
        {
            printf("[INFO]: Obstacle border collision detected\n");
            CGAL::difference(inexact2exact(border), inexact2exact(p), std::back_inserter(res));
            assert(res.size() == 1);
            border = exact2inexact(res.front().outer_boundary());
            obstacles.erase(obstacles.begin() + i);
            i--;
        }
    }

    // If gate falls outside of newly offset border
    if (border.has_on_unbounded_side(gate.source()))
    {
        double record = INFINITY;
        Point_2 closest;
        for (size_t i = 0; i < border.size(); i++)
        {
            const auto ix = CGAL::intersection(gate.opposite(), border.edge(i));
            if (ix)
            {
                const Point_2 *ix_p = boost::get<Point_2>(&*ix);
                if (ix_p)
                {
                    double dsq = CGAL::squared_distance(gate.source(), *ix_p);
                    if (dsq < record)
                    {
                        record = dsq;
                        closest = *ix_p;
                    }
                }
            }
        }
        auto offset = gate.opposite().to_vector();
        offset = 0.001 * offset / std::sqrt(offset.squared_length());
        gateProjection = closest + offset;
    }
}

int getVertexIndexOffset(Polygon_2 &p1, Polygon_2 &p2)
{
    // std::map<int, int> votes;
    // std::array<int, 4> dists;
    // int N = p1.size();
    // dists.at(0) = distance(p1.begin(), p1.top_vertex()) - distance(p2.begin(), p2.top_vertex());
    // dists.at(1) = distance(p1.begin(), p1.right_vertex()) - distance(p2.begin(), p2.right_vertex());
    // dists.at(2) = distance(p1.begin(), p1.left_vertex()) - distance(p2.begin(), p1.left_vertex());
    // dists.at(3) = distance(p1.begin(), p1.bottom_vertex()) - distance(p2.begin(), p2.bottom_vertex());
    // for (int d: dists) {
    //     int d_mod = (d%N+N)%N;
    //     if (votes.count(d_mod))
    //         votes[d_mod]++;
    //     else
    //         votes[d_mod] = 1;
    // }
    // int record = 0;
    // int res = 0;
    // for (auto it = votes.begin(); it != votes.end(); it++) {
    //     if (it->second > record) {
    //         record = it->second;
    //         res = it->first;
    //     }
    // }
    // return dists.at(0);
    return distance(p1.begin(), p1.right_vertex()) - distance(p2.begin(), p2.right_vertex());
}

int mod(int a, int b)
{
    return (a % b + b) % b;
}

std::vector<Point_2> Map::getReflexVertices(std::vector<Point_2> &offsetRVs, std::vector<double> &inboundAngs, std::vector<double> &outboundAngs)
{
    std::vector<Point_2> rv;
    offsetRVs = std::vector<Point_2>{};
    inboundAngs = std::vector<double>{};
    outboundAngs = std::vector<double>{};
    size_t n = border.size();
    Polygon_2 offsetBorder = offsetPolygon(border, -0.001);
    int borderIdxOffset = getVertexIndexOffset(border, offsetBorder);
    for (size_t i = 0; i < n; i++)
    {
        auto p = border.edge(i).to_vector();
        auto q = border.edge(mod(i + 1, n)).to_vector();
        if (p.x() * q.y() - p.y() * q.x() < 0)
        {
            double inAng = dir2ang(q.direction());
            double outAng = dir2ang(p.direction());
            inboundAngs.push_back(inAng);
            outboundAngs.push_back(outAng);
            rv.push_back(border.vertex(mod(i + 1, n)));
            offsetRVs.push_back(offsetBorder.vertex(mod(-borderIdxOffset + i + 1, n)));
        }
    }
    for (auto &o : obstacles)
    {
        Polygon_2 poly = o.getPoly(10);
        Polygon_2 offsetPoly = offsetPolygon(poly, 0.001);
        int idxOffset = getVertexIndexOffset(poly, offsetPoly);
        size_t n = poly.size();
        for (size_t i = 0; i < n; i++)
        {
            auto p = poly.edge(i).to_vector();
            auto q = poly.edge(mod(i + 1, n)).to_vector();
            if (p.x() * q.y() - p.y() * q.x() > 0)
            {
                double inAng = dir2ang(p.direction());
                double outAng = dir2ang(q.direction());
                inboundAngs.push_back(inAng);
                outboundAngs.push_back(outAng);
                rv.push_back(poly.vertex(mod(i + 1, n)));
                offsetRVs.push_back(offsetPoly.vertex(mod(-idxOffset + i + 1, n)));
            }
        }
    }
    return rv;
}

std::vector<Point_2> Map::visibilityQuery(Point_2 queryPoint)
{
    if (!visibilityCacheComputed)
    {
        CGAL::insert_non_intersecting_curves(visibilityCache, border.edges_begin(), border.edges_end());
        for (auto &o : obstacles)
        {
            Polygon_2 p = o.getPoly(10);
            CGAL::insert_non_intersecting_curves(visibilityCache, p.edges_begin(), p.edges_end());
        }
        bool face_found = false;
        for (auto fit = visibilityCache.faces_begin(); fit != visibilityCache.faces_end(); ++fit)
        {
            if (!fit->is_unbounded())
            {
                auto ccb = fit->outer_ccb();
                if (std::find(border.edges_begin(), border.edges_end(), ccb->curve()) != border.edges_end())
                {
                    main_face = fit;
                    face_found = true;
                    break;
                }
            }
        }
        if (!face_found)
            printf("[WARNING]: Could not find face for visibility queries\n");

        visibilityCacheComputed = true;
    }
    std::vector<Point_2> vec{};
    Visibility_2 visibility(visibilityCache);
    Arrangement_2 vis_output;
    visibility.compute_visibility(queryPoint, main_face, vis_output);
    for (Edge_const_iterator eit = vis_output.edges_begin(); eit != vis_output.edges_end(); ++eit)
    {
        vec.push_back(eit->target()->point());
    }
    return vec;
}