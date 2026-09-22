#include "coaster/simulation.hpp"
#include <unordered_map>

namespace coaster {
namespace {
// A car/occupied containment volume in metres, including its undercarriage.
// The continuous bounds below sweep the whole box, not only its corners.
struct Box {
    Vec3 center;
    std::array<Vec3, 3> axis;
    std::array<double, 3> half;
};
struct Sweep {
    Box box;
    double begin, end;
    std::size_t span;
};
double choose(int n, int k) {
    double r = 1;
    for (int i = 1; i <= k; ++i)
        r *= double(n - k + i) / i;
    return r;
}
template <std::size_t N>
std::vector<Vec3> bernstein(const std::array<Vec3, N> &power, int derivative, double scale) {
    const int degree = int(N) - 1 - derivative;
    std::vector<Vec3> result(std::size_t(degree + 1));
    for (int i = 0; i <= degree; ++i)
        for (int k = 0; k <= i; ++k) {
            double f = 1;
            for (int j = 1; j <= derivative; ++j)
                f *= k + j;
            result[std::size_t(i)] += power[std::size_t(k + derivative)] *
                                      (f * choose(i, k) / choose(degree, k) / std::pow(scale, derivative));
        }
    return result;
}
double maximumNorm(const std::vector<Vec3> &b) {
    double r = 0;
    for (auto v : b)
        r = std::max(r, norm(v));
    return r;
}
// Convex-hull bounds on polynomial derivatives, followed by the exact
// normalization/projection derivative inequalities used by Track::at.
double sweepDerivative(const Track &track, std::size_t index) {
    const auto &s = track.spans[index];
    const auto center = track.at(s.begin + s.length / 2);
    const auto first = bernstein(s.p, 1, s.length);
    double minimum = 1e30;
    for (auto v : first)
        minimum = std::min(minimum, dot(v, center.t));
    if (minimum <= .5)
        throw std::runtime_error("Cannot bound degenerate track derivative");
    const double forward = maximumNorm(first), turn = maximumNorm(bernstein(s.p, 2, s.length)) / minimum;
    const double raw = maximumNorm(bernstein(s.u, 0, 1)),
                 rawDerivative = maximumNorm(bernstein(s.u, 1, s.length));
    const double projectedDerivative = rawDerivative + 2 * raw * turn;
    // Canonical raw up is unit and orthogonal at both shared endpoint jets.
    Vec3 endUp, endFirst;
    for (auto u : s.u)
        endUp += u;
    for (std::size_t k = 1; k < s.p.size(); ++k)
        endFirst += s.p[k] * (double(k) / s.length);
    const Vec3 t0 = unit(s.p[1]), t1 = unit(endFirst);
    const double endpointMinimum =
        std::min(norm(s.u[0] - t0 * dot(s.u[0], t0)), norm(endUp - t1 * dot(endUp, t1)));
    const double minimumUp = endpointMinimum - projectedDerivative * s.length / 2 - 1e-9;
    if (minimumUp <= .5)
        throw std::runtime_error("Cannot bound degenerate orientation derivative");
    const double up = projectedDerivative / minimumUp;
    return forward + 3.25 * turn + 4.35 * up + 1e-9;
}
Sweep enclose(const Track &track, std::size_t span, double a, double b, double derivative) {
    const auto f = track.at((a + b) / 2);
    const double radius = derivative * (b - a) / 2;
    return {{f.p + f.u * 1.05, {f.t, f.r, f.u}, {1.9 + radius, 1.35 + radius, 1.95 + radius}}, a, b, span};
}
Vec3 extent(const Box &b) {
    Vec3 e;
    for (std::size_t i = 0; i < 3; ++i) {
        const auto a = b.axis[i];
        e += Vec3{std::abs(a.x), std::abs(a.y), std::abs(a.z)} * b.half[i];
    }
    return e;
}
bool terrainClear(const Box &box, double plateau, double &certifiedGap, int depth = 0) {
    const auto e = extent(box);
    const double gap = box.center.z - e.z - terrainUpperBound(box.center - e, box.center + e, plateau);
    if (gap >= .25) {
        certifiedGap = std::min(certifiedGap, gap);
        return true;
    }
    if (depth >= 9)
        return false;
    // Split the actual occupied volume, retaining its orientation. This
    // avoids treating a tall vertical car's whole AABB as filled terrain.
    std::size_t axis = 0;
    double score = 0;
    for (std::size_t i = 0; i < 3; ++i) {
        const auto v = box.axis[i];
        const double candidate = box.half[i] * (std::abs(v.x) + std::abs(v.y) + std::abs(v.z));
        if (candidate > score) {
            score = candidate;
            axis = i;
        }
    }
    Box a = box, b = box;
    a.half[axis] *= .5;
    b.half[axis] *= .5;
    a.center += a.axis[axis] * a.half[axis];
    b.center = b.center - b.axis[axis] * b.half[axis];
    double local = 1e9;
    if (!terrainClear(a, plateau, local, depth + 1) || !terrainClear(b, plateau, local, depth + 1))
        return false;
    certifiedGap = std::min(certifiedGap, local);
    return true;
}
double separation(const Box &a, const Box &b) {
    const Vec3 delta = b.center - a.center;
    double best = -1e9;
    auto axis = [&](Vec3 v) {
        const double n = norm(v);
        if (n < 1e-9)
            return;
        v = v / n;
        double radius = 0;
        for (std::size_t i = 0; i < 3; ++i)
            radius += std::abs(dot(a.axis[i], v)) * a.half[i] + std::abs(dot(b.axis[i], v)) * b.half[i];
        best = std::max(best, std::abs(dot(delta, v)) - radius);
    };
    for (auto v : a.axis)
        axis(v);
    for (auto v : b.axis)
        axis(v);
    for (auto x : a.axis)
        for (auto y : b.axis)
            axis(cross(x, y));
    return best;
}
bool separated(const Track &track, const Sweep &a, const Sweep &b, const std::vector<double> &derivative,
               double &gap, int depth = 0) {
    const double distance = separation(a.box, b.box);
    if (distance >= .25) {
        gap = std::min(gap, distance);
        return true;
    }
    if (depth >= 10)
        return false;
    const Sweep &larger = a.end - a.begin >= b.end - b.begin ? a : b;
    const bool left = &larger == &a;
    const double mid = (larger.begin + larger.end) / 2;
    const auto c = enclose(track, larger.span, larger.begin, mid, derivative[larger.span]),
               d = enclose(track, larger.span, mid, larger.end, derivative[larger.span]);
    double local = 1e9;
    const bool okay = left ? (separated(track, c, b, derivative, local, depth + 1) &&
                              separated(track, d, b, derivative, local, depth + 1))
                           : (separated(track, a, c, derivative, local, depth + 1) &&
                              separated(track, a, d, derivative, local, depth + 1));
    if (okay)
        gap = std::min(gap, local);
    return okay;
}
struct Cell {
    int x, y, z;
    bool operator==(const Cell &) const = default;
};
struct Hash {
    std::size_t operator()(Cell c) const {
        return std::hash<int>{}(c.x) ^ (std::hash<int>{}(c.y) << 1) ^ (std::hash<int>{}(c.z) << 2);
    }
};
} // namespace
Clearance assessClearance(const Track &track, double plateau, double spacing, const Cancel &cancel) {
    if (!(spacing > 0 && spacing <= 2))
        throw std::runtime_error("Invalid swept clearance resolution");
    Clearance result;
    double ordinary = 0, ordinaryLength = 0;
    std::vector<Sweep> sweeps;
    std::vector<double> derivative;
    derivative.reserve(track.spans.size());
    for (std::size_t i = 0; i < track.spans.size(); ++i)
        derivative.push_back(sweepDerivative(track, i));
    for (std::size_t i = 0; i < track.spans.size(); ++i) {
        const auto &span = track.spans[i];
        const int count = std::max(1, int(std::ceil(span.length / spacing)));
        for (int j = 0; j < count; ++j) {
            poll(cancel);
            const double a = span.begin + span.length * j / count,
                         b = span.begin + span.length * (j + 1) / count;
            const auto sweep = enclose(track, i, a, b, derivative[i]);
            double gap = 1e9;
            bool clear = terrainClear(sweep.box, plateau, gap);
            if (!clear) {
                clear = true;
                for (int k = 0; k < 8; ++k) {
                    const auto refined = enclose(track, i, std::lerp(a, b, k / 8.),
                                                 std::lerp(a, b, (k + 1) / 8.), derivative[i]);
                    if (!terrainClear(refined.box, plateau, gap)) {
                        clear = false;
                        break;
                    }
                }
            }
            if (!clear)
                ++result.terrainHits;
            else
                result.minimumGround = std::min(result.minimumGround, gap);
            const auto f = track.at((a + b) / 2);
            const auto role = track.source[f.element].role;
            if (role == Role::Clifftop || role == Role::Journey || role == Role::Launch ||
                role == Role::Terminal) {
                const double above = f.p.z - ground(f.p.x, f.p.y, plateau);
                ordinary += above * (b - a);
                ordinaryLength += b - a;
                result.ordinaryMaxHeight = std::max(result.ordinaryMaxHeight, above);
            }
            if (std::hypot(sweep.box.half[0], sweep.box.half[1], sweep.box.half[2]) > 5.8)
                throw std::runtime_error("Swept broad phase needs finer intervals");
            sweeps.push_back(sweep);
            ++result.samples;
        }
    }
    std::unordered_map<Cell, std::vector<std::size_t>, Hash> grid;
    for (std::size_t i = 0; i < sweeps.size(); ++i) {
        poll(cancel);
        const auto &a = sweeps[i];
        const auto p = a.box.center;
        const Cell cell{int(std::floor(p.x / 12)), int(std::floor(p.y / 12)), int(std::floor(p.z / 12))};
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = -1; z <= 1; ++z) {
                    const auto found = grid.find({cell.x + x, cell.y + y, cell.z + z});
                    if (found == grid.end())
                        continue;
                    for (auto index : found->second) {
                        const auto &b = sweeps[index];
                        const double local = a.begin - b.end, wrapped = track.length - a.end + b.begin;
                        if (local <= 6 || (track.closed && wrapped <= 6))
                            continue;
                        double gap = 1e9;
                        if (!separated(track, a, b, derivative, gap)) {
                            ++result.trackHits;
                            if (result.firstTrackS < 0) {
                                result.firstTrackS = (a.begin + a.end) / 2;
                                result.secondTrackS = (b.begin + b.end) / 2;
                            }
                        } else
                            result.minimumNonlocal = std::min(result.minimumNonlocal, gap);
                    }
                }
        grid[cell].push_back(i);
    }
    result.ordinaryMeanHeight = ordinaryLength ? ordinary / ordinaryLength : 0;
    result.continuous = true;
    return result;
}
} // namespace coaster
