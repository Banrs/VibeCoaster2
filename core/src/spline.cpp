#include "coaster/spline.hpp"

namespace coaster {
namespace {
struct D3 {
    double a{}, b{}, c{}, d{};
};
D3 operator+(D3 x, D3 y) {
    return {x.a + y.a, x.b + y.b, x.c + y.c, x.d + y.d};
}
D3 operator-(D3 x, D3 y) {
    return {x.a - y.a, x.b - y.b, x.c - y.c, x.d - y.d};
}
D3 operator*(D3 x, D3 y) {
    return {x.a * y.a, x.b * y.a + x.a * y.b, x.c * y.a + 2 * x.b * y.b + x.a * y.c,
            x.d * y.a + 3 * x.c * y.b + 3 * x.b * y.c + x.a * y.d};
}
D3 operator*(D3 x, double k) {
    return {x.a * k, x.b * k, x.c * k, x.d * k};
}
D3 sine(D3 x) {
    const double s = std::sin(x.a), c = std::cos(x.a);
    return {s, c * x.b, c * x.c - s * x.b * x.b, c * x.d - 3 * s * x.b * x.c - c * x.b * x.b * x.b};
}
D3 cosine(D3 x) {
    return sine(x + D3{pi / 2});
}
D3 sampleAngle(const std::vector<GeometryControl> &controls, double s, std::size_t axis) {
    auto hi = std::upper_bound(controls.begin(), controls.end(), s,
                               [](double x, const GeometryControl &k) { return x < k.s; });
    if (hi == controls.begin()) {
        const auto &k = controls.front();
        return {k.value[axis], k.first[axis], k.second[axis], k.third[axis]};
    }
    if (hi == controls.end()) {
        const auto &k = controls.back();
        return {k.value[axis], k.first[axis], k.second[axis], k.third[axis]};
    }
    const auto &a = *(hi - 1);
    const auto &b = *hi;
    const double h = b.s - a.s, u = (s - a.s) / h;
    const double c0 = a.value[axis], c1 = a.first[axis] * h, c2 = a.second[axis] * h * h / 2,
                 c3 = a.third[axis] * h * h * h / 6;
    const double p = b.value[axis] - c0 - c1 - c2 - c3, v = b.first[axis] * h - c1 - 2 * c2 - 3 * c3,
                 d = b.second[axis] * h * h - 2 * c2 - 6 * c3, j = b.third[axis] * h * h * h - 6 * c3;
    const std::array<double, 8> c{c0,
                                  c1,
                                  c2,
                                  c3,
                                  35 * p - 15 * v + 2.5 * d - j / 6,
                                  -84 * p + 39 * v - 7 * d + j / 2,
                                  70 * p - 34 * v + 6.5 * d - j / 2,
                                  -20 * p + 10 * v - 2 * d + j / 6};
    D3 q{c[7]};
    for (int i = 6; i >= 0; --i)
        q = {q.a * u + c[std::size_t(i)], q.b * u + q.a, q.c * u + 2 * q.b, q.d * u + 3 * q.c};
    return {q.a, q.b / h, q.c / (h * h), q.d / (h * h * h)};
}
struct Basis {
    std::array<Vec3, 4> t, u;
    double drive{};
};
Basis basis(const Program &p, double s) {
    const auto pitch = sampleAngle(p.geometry, s, 0), yaw = sampleAngle(p.geometry, s, 1),
               bank = sampleAngle(p.geometry, s, 2);
    const auto cp = cosine(pitch), sp = sine(pitch), cy = cosine(yaw), sy = sine(yaw), cb = cosine(bank),
               sb = sine(bank);
    const std::array<D3, 3> t{cp * cy, cp * sy, sp},
        u{sp * cy * cb * (-1) + sy * sb, sp * sy * cb * (-1) - cy * sb, cp * cb};
    Basis result;
    for (std::size_t j = 0; j < 4; ++j) {
        auto value = [&](D3 d) { return j == 0 ? d.a : j == 1 ? d.b : j == 2 ? d.c : d.d; };
        result.t[j] = {value(t[0]), value(t[1]), value(t[2])};
        result.u[j] = {value(u[0]), value(u[1]), value(u[2])};
    }
    result.drive = sampleAngle(p.geometry, s, 3).a;
    return result;
}
struct GS {
    Vec3 p;
    double v2{}, time{}, loss{}, drive{};
};
GS operator+(GS a, GS b) {
    return {a.p + b.p, a.v2 + b.v2, a.time + b.time, a.loss + b.loss, a.drive + b.drive};
}
GS operator*(GS a, double k) {
    return {a.p * k, a.v2 * k, a.time * k, a.loss * k, a.drive * k};
}
GS derivative(const Program &p, const GS &q, double s) {
    if (q.v2 < .25)
        throw std::runtime_error("Spline exceeds available kinetic energy");
    const auto b = basis(p, s);
    const double v = std::sqrt(q.v2), loss = p.rolling * std::tanh(v / .2) + p.drag * q.v2;
    return {b.t[0], 2 * (b.drive - gravity * b.t[0].z - loss), 1 / v, loss, b.drive};
}
GS advanceSpline(const Program &p, const GS &q, double s, double ds) {
    const auto a = derivative(p, q, s), b = derivative(p, q + a * (ds / 2), s + ds / 2),
               c = derivative(p, q + b * (ds / 2), s + ds / 2), d = derivative(p, q + c * ds, s + ds);
    return q + (a + b * 2 + c * 2 + d) * (ds / 6);
}
Jet splineJet(const Program &p, const GS &q, double s) {
    const auto b = basis(p, s);
    return {{q.p, b.t[0], b.t[1], b.t[2], b.t[3]}, b.u, p.initial.s + s, q.time, std::sqrt(q.v2), b.drive, 0};
}
std::pair<Shot, std::vector<Jet>> integrate(const Program &p, double step, bool retain,
                                            const Cancel &cancel) {
    validateSpline(p);
    GS state{p.initial.p, p.initial.v * p.initial.v, 0, p.initial.lossWork, p.initial.driveWork};
    Shot shot;
    shot.maximumHeight = shot.minimumHeight = p.initial.p.z;
    std::vector<Jet> jets;
    if (retain)
        jets.push_back(splineJet(p, state, 0));
    const double length = p.geometry.back().s;
    const int count = std::max(1, int(std::ceil(length / step)));
    const double ds = length / count;
    for (int i = 0; i < count; ++i) {
        if ((i & 63) == 0)
            poll(cancel);
        state = advanceSpline(p, state, i * ds, ds);
        shot.maximumHeight = std::max(shot.maximumHeight, state.p.z);
        shot.minimumHeight = std::min(shot.minimumHeight, state.p.z);
        if (retain)
            jets.push_back(splineJet(p, state, (i + 1) * ds));
    }
    const auto b = basis(p, length);
    shot.end = {state.p, b.t[0], b.u[0], std::sqrt(state.v2), p.initial.s + length, state.loss, state.drive};
    shot.yaw = p.geometry.back().value[1];
    shot.pitchTurn = p.geometry.back().value[0];
    return {shot, std::move(jets)};
}
double S(double u) {
    u = std::clamp(u, 0., 1.);
    return std::pow(u, 5) * (126 + u * (-420 + u * (540 + u * (-315 + 70 * u))));
}
double rollEase(double u) {
    u = std::clamp(u, 0., 1.);
    // C3 endpoint continuity with a lower peak roll speed than the ninth-
    // degree placement ease. The roll still covers one complete revolution.
    return u * u * u * u * (35 + u * (-84 + u * (70 - 20 * u)));
}
double Sd(double u) {
    u = std::clamp(u, 0., 1.);
    return 630 * std::pow(u * (1 - u), 4);
}
} // namespace
void validateSpline(const Program &p) {
    if (p.geometry.size() < 2 || p.geometry.size() > 2000 || p.geometry.front().s != 0 ||
        p.geometry.back().s > 4000 || p.initial.v < 1 || !finite(p.initial.p))
        throw std::runtime_error("Invalid spatial spline source");
    double previous = -1;
    for (const auto &k : p.geometry) {
        if (!std::isfinite(k.s) || k.s <= previous)
            throw std::runtime_error("Invalid spline parameter");
        previous = k.s;
        for (const auto &values : {k.value, k.first, k.second, k.third})
            for (double x : values)
                if (!std::isfinite(x) || std::abs(x) > 10000)
                    throw std::runtime_error("Invalid spline jet");
    }
    const auto start = basis(p, 0);
    if (norm(start.t[0] - p.initial.t) > 1e-5 || norm(start.u[0] - p.initial.u) > 1e-5)
        throw std::runtime_error("Spline entry frame differs from inherited live port");
}
Shot shootSpline(const Program &p, double step, const Cancel &cancel) {
    return integrate(p, step, false, cancel).first;
}
std::vector<Jet> replaySpline(const Program &p, double step, const Cancel &cancel) {
    return integrate(p, step, true, cancel).second;
}
Program terrainSpline(const State &state, const SplineIntent &input, const Cancel &cancel) {
    if (input.length < 20 || input.length > 3000 || std::abs(input.headingChange) > 2 * pi)
        throw std::runtime_error("Unsupported spline intent");
    Program p;
    p.initial = state;
    const double length = input.length, heading = std::atan2(state.t.y, state.t.x),
                 entry = std::asin(state.t.z);
    auto build = [&](double rise, double descent) {
        auto pitchAt = [&](double s) {
            const double u = std::clamp(s / length, 0., 1.);
            return entry + (input.exitPitch - entry) * S(u) +
                   rise * std::sin(2 * pi * u) * std::pow(std::sin(pi * u), 4) +
                   descent * std::pow(std::sin(pi * u), 4);
        };
        auto value = [&](double s) {
            const double u = std::clamp(s / length, 0., 1.);
            const double pitch = pitchAt(s), yaw = heading + input.headingChange * S(u),
                         yawS = input.headingChange * Sd(u) / length;
            const double speed2 = std::max(25., state.v * state.v - 2 * gravity * input.exitHeight * S(u));
            const double bank = -std::atan2(speed2 * std::cos(pitch) * yawS, gravity * std::cos(pitch)) +
                                input.bankBias * std::pow(std::sin(pi * u), 4) +
                                input.rollTurns * 2 * pi * rollEase(u);
            return std::array<double, 4>{pitch, yaw, bank, 0};
        };
        p.geometry.clear();
        const int count = std::max(20, int(std::ceil(length / 5)));
        const double h = .1;
        for (int i = 0; i <= count; ++i) {
            const double s = length * i / count;
            const auto v = value(s), a = value(s - h), b = value(s + h), aa = value(s - 2 * h),
                       bb = value(s + 2 * h);
            GeometryControl k;
            k.s = s;
            k.value = v;
            for (std::size_t j = 0; j < 4; ++j) {
                k.first[j] = (b[j] - a[j]) / (2 * h);
                k.second[j] = (b[j] - 2 * v[j] + a[j]) / (h * h);
                k.third[j] = (bb[j] - 2 * b[j] + 2 * a[j] - aa[j]) / (2 * h * h * h);
            }
            if (i == 0 || i == count)
                k.first = k.second = k.third = {};
            p.geometry.push_back(k);
        }
    };
    if (input.height == 0 && input.exitHeight == 0 && std::abs(entry) < 1e-6 && input.exitPitch == 0)
        build(0, 0);
    else if (input.height == 0 && input.exitHeight < 0 && std::abs(entry) < 1e-6 && input.exitPitch == 0) {
        std::array<double, 1> parameter{input.exitHeight * 3 / length};
        auto residual = [&](const auto &q) {
            build(0, q[0]);
            return std::array<double, 1>{(shootSpline(p, 2, cancel).end.p.z - state.p.z - input.exitHeight) /
                                         30};
        };
        if (!solve<1>(parameter, {{{-.8, 0}}}, residual))
            throw std::runtime_error("Descending spline cannot reach its exit");
        build(0, parameter[0]);
    } else {
        std::array<double, 2> parameters{input.height * 8 / length, input.exitHeight * 3 / length};
        auto residual = [&](const auto &q) {
            build(q[0], q[1]);
            const auto s = shootSpline(p, 2, cancel);
            return std::array<double, 2>{(s.maximumHeight - state.p.z - input.height) / 30,
                                         (s.end.p.z - state.p.z - input.exitHeight) / 30};
        };
        if (!solve<2>(parameters, {{{-1.2, 1.2}, {-.8, .8}}}, residual))
            throw std::runtime_error("Spline cannot reach authored height and exit");
        build(parameters[0], parameters[1]);
    }
    const auto jets = replaySpline(p, 1, cancel);
    p.geometricDuration = jets.back().time;
    return p;
}
Program splineTo(const State &state, Vec3 target, double rollTurns, const Cancel &cancel) {
    const Vec3 delta = target - state.p;
    if (!finite(target) || !std::isfinite(rollTurns))
        throw std::runtime_error("Invalid fixed spline destination");
    const double bearing = std::atan2(delta.y, delta.x), heading = std::atan2(state.t.y, state.t.x);
    const double turn = 2 * std::remainder(bearing - heading, 2 * pi);
    std::array<double, 2> parameters{std::hypot(delta.x, delta.y) * 1.03, turn};
    Program result;
    auto build = [&](const auto &q) {
        return terrainSpline(state, {q[0], q[1], 0, delta.z, 0, 0, 0, rollTurns}, cancel);
    };
    auto residual = [&](const auto &q) {
        poll(cancel);
        const auto end = shootSpline(build(q), 1, cancel).end.p;
        return std::array<double, 2>{(end.x - target.x) / 100, (end.y - target.y) / 100};
    };
    if (!solve<2>(parameters, {{{100, 1200}, {-1.5, 1.5}}}, residual))
        throw std::runtime_error("Spline cannot reach its fixed site destination");
    result = build(parameters);
    if (norm(shootSpline(result, .5, cancel).end.p - target) > .005)
        throw std::runtime_error("Fixed spline destination did not converge");
    return result;
}
} // namespace coaster
