#include "coaster/terrain.hpp"
#include <limits>
#include <string>

namespace coaster {
namespace {
std::vector<double> coordinates(double first, double last) {
    constexpr double horizon = 20000, nearStep = 5;
    std::vector<double> result, left;
    double at = first, step = nearStep;
    while (at > -horizon) {
        at = std::max(-horizon, at - step);
        left.push_back(at);
        step = std::min(500., step * 1.17);
    }
    result.assign(left.rbegin(), left.rend());
    for (at = first; at <= last; at += nearStep)
        result.push_back(at);
    at = last;
    step = nearStep;
    while (at < horizon) {
        at = std::min(horizon, at + step);
        result.push_back(at);
        step = std::min(500., step * 1.17);
    }
    return result;
}
std::size_t cell(const std::vector<double> &axis, double value) {
    if (!std::isfinite(value) || value < axis.front() || value > axis.back())
        throw std::runtime_error("Track leaves the fixed terrain domain");
    const auto hi = std::upper_bound(axis.begin(), axis.end(), value);
    return std::min(axis.size() - 2, std::size_t(hi - axis.begin() - 1));
}
struct Polygon {
    std::array<Vec3, 8> points{};
    std::size_t size{};
};
Polygon clipped(const Polygon &input, int axis, double boundary, bool greater) {
    Polygon out;
    if (!input.size)
        return out;
    auto distance = [&](Vec3 p) { return ((axis == 0 ? p.x : p.y) - boundary) * (greater ? 1 : -1); };
    auto append = [&](Vec3 p) {
        if (out.size && norm(p - out.points[out.size - 1]) < 1e-12)
            return;
        if (out.size >= out.points.size())
            throw std::runtime_error("Terrain clipping bound exceeded");
        out.points[out.size++] = p;
    };
    Vec3 prior = input.points[input.size - 1];
    double previous = distance(prior);
    for (std::size_t i = 0; i < input.size; ++i) {
        const Vec3 current = input.points[i];
        const double next = distance(current);
        if ((previous >= 0) != (next >= 0)) {
            const double fraction = std::clamp(previous / (previous - next), 0., 1.);
            append(prior + (current - prior) * fraction);
        }
        if (next >= 0) {
            append(current);
        }
        prior = current;
        previous = next;
    }
    if (out.size > 1 && norm(out.points[0] - out.points[out.size - 1]) < 1e-12)
        --out.size;
    return out;
}
} // namespace
const std::vector<double> &terrainXCoordinates() {
    static const auto values = coordinates(-800, 1500);
    return values;
}
const std::vector<double> &terrainYCoordinates() {
    static const auto values = coordinates(-1650, 800);
    return values;
}
double siteElevation(double x, double y, double plateau) {
    const double shoulder = 18 + 332 * smooth((std::abs(x) - 250) / 275);
    const double q = std::clamp((y + shoulder) / (2 * shoulder), 0., 1.);
    const double terrace = plateau * std::pow(q, 5) * (126 + q * (-420 + q * (540 + q * (-315 + 70 * q))));
    return terrace + 1.4 * std::sin(x / 270) * std::sin(y / 220) + .8 * std::sin((x + y) / 410);
}
double ground(double x, double y, double plateau) {
    const auto &xs = terrainXCoordinates();
    const auto &ys = terrainYCoordinates();
    const auto i = cell(xs, x), j = cell(ys, y);
    const double u = (x - xs[i]) / (xs[i + 1] - xs[i]), v = (y - ys[j]) / (ys[j + 1] - ys[j]);
    const double a = siteElevation(xs[i], ys[j], plateau), c = siteElevation(xs[i + 1], ys[j + 1], plateau);
    if (v <= u) {
        const double b = siteElevation(xs[i + 1], ys[j], plateau);
        return a * (1 - u) + b * (u - v) + c * v;
    }
    const double d = siteElevation(xs[i], ys[j + 1], plateau);
    return a * (1 - v) + c * u + d * (v - u);
}
Vec3 terrainShadingNormal(double x, double y, double plateau) {
    constexpr double h = .2;
    const double dx = (siteElevation(x + h, y, plateau) - siteElevation(x - h, y, plateau)) / (2 * h);
    const double dy = (siteElevation(x, y + h, plateau) - siteElevation(x, y - h, plateau)) / (2 * h);
    return unit({-dx, -dy, 1});
}
double terrainUpperBound(Vec3 lo, Vec3 hi, double plateau) {
    if (lo.x > hi.x || lo.y > hi.y)
        throw std::runtime_error("Invalid terrain query rectangle");
    const auto &xs = terrainXCoordinates();
    const auto &ys = terrainYCoordinates();
    const auto x0 = cell(xs, lo.x), x1 = cell(xs, hi.x), y0 = cell(ys, lo.y), y1 = cell(ys, hi.y);
    double maximum = -std::numeric_limits<double>::infinity();
    for (std::size_t i = x0; i <= x1; ++i)
        for (std::size_t j = y0; j <= y1; ++j) {
            const Vec3 a{xs[i], ys[j], siteElevation(xs[i], ys[j], plateau)},
                b{xs[i + 1], ys[j], siteElevation(xs[i + 1], ys[j], plateau)},
                c{xs[i + 1], ys[j + 1], siteElevation(xs[i + 1], ys[j + 1], plateau)},
                d{xs[i], ys[j + 1], siteElevation(xs[i], ys[j + 1], plateau)};
            auto includeTriangle = [&](Vec3 first, Vec3 second, Vec3 third) {
                Polygon p;
                p.points[0] = first;
                p.points[1] = second;
                p.points[2] = third;
                p.size = 3;
                p = clipped(p, 0, lo.x, true);
                p = clipped(p, 0, hi.x, false);
                p = clipped(p, 1, lo.y, true);
                p = clipped(p, 1, hi.y, false);
                for (std::size_t k = 0; k < p.size; ++k)
                    maximum = std::max(maximum, p.points[k].z);
            };
            includeTriangle(a, b, c);
            includeTriangle(a, c, d);
        }
    if (!std::isfinite(maximum))
        throw std::runtime_error("Empty terrain bound: " + std::to_string(lo.x) + "," + std::to_string(lo.y) +
                                 " to " + std::to_string(hi.x) + "," + std::to_string(hi.y) + " cells " +
                                 std::to_string(x0) + "," + std::to_string(y0) + " to " + std::to_string(x1) +
                                 "," + std::to_string(y1));
    // Includes conservative render-vertex quantization allowance (one millimetre).
    return maximum + .001;
}
} // namespace coaster
