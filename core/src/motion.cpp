#include "coaster/motion.hpp"
#include "coaster/spline.hpp"
#include <sstream>

namespace coaster {
namespace {
constexpr Vec3 g{0, 0, -gravity};
struct ScalarJet {
    double v{}, d{}, dd{};
};
ScalarJet interpolate(double a, double ad, double add, double b, double bd, double bdd, double h, double x) {
    const double c0 = a, c1 = ad * h, c2 = add * h * h / 2, p = b - c0 - c1 - c2, v = bd * h - c1 - 2 * c2,
                 d = bdd * h * h - 2 * c2;
    const std::array<double, 6> c{
        c0, c1, c2, 10 * p - 4 * v + d / 2, -15 * p + 7 * v - d, 6 * p - 3 * v + d / 2};
    double value = c[5], first = 0, second = 0;
    for (int i = 4; i >= 0; --i) {
        second = second * x + 2 * first;
        first = first * x + value;
        value = value * x + c[std::size_t(i)];
    }
    return {value, first / h, second / (h * h)};
}
double loss(const Program &p, double v) {
    return p.rolling * std::tanh(v / .2) + p.drag * v * v;
}
struct D {
    Vec3 omega;
    double dv{};
};
D derivative(const State &s, const Program &p, double time) {
    if (!(s.v >= 0 && s.v < 200))
        throw std::runtime_error("Motion stalls or exceeds supported speed");
    const auto c = controlAt(p, time);
    const Vec3 r = cross(s.t, s.u);
    const Vec3 a = g + s.u * (gravity * c.normal) + r * (gravity * c.lateral);
    const Vec3 transverse = a - s.t * dot(a, s.t);
    if (s.v < .1 && norm(transverse) > 1e-8)
        throw std::runtime_error("Curved source at zero speed");
    const Vec3 td = s.v < .1 ? Vec3{} : transverse / s.v;
    double twist = c.twist;
    if (p.gravityBank) {
        const double h = s.t.x * s.t.x + s.t.y * s.t.y;
        if (h < 1e-5)
            throw std::runtime_error("Bank reference at vertical tangent");
        twist += s.t.z * (s.t.x * td.y - s.t.y * td.x) / h;
    }
    return {cross(s.t, td) + s.t * twist, c.drive + dot(g, s.t) - loss(p, s.v)};
}
struct Q {
    double w{1};
    Vec3 v{};
    Q operator+(Q b) const { return {w + b.w, v + b.v}; }
    Q operator*(double a) const { return {w * a, v * a}; }
};
Q normalized(Q q) {
    const double d = std::sqrt(q.w * q.w + dot(q.v, q.v));
    if (!(d > .1))
        throw std::runtime_error("Invalid rotation");
    return q * (1 / d);
}
Vec3 turned(Vec3 v, Q q) {
    q = normalized(q);
    return v + cross(q.v, v) * (2 * q.w) + cross(q.v, cross(q.v, v)) * 2;
}
double pitch(Vec3 t) {
    return std::atan2(t.z, std::hypot(t.x, t.y));
}
} // namespace
const char *roleName(Role r) {
    static constexpr const char *names[] = {"launch",
                                            "opening",
                                            "ascent-lsm",
                                            "clifftop-active",
                                            "edge-exposure",
                                            "lip",
                                            "cliff",
                                            "downhill-lsm",
                                            "protected-camelback",
                                            "wave-180",
                                            "yawing-loop",
                                            "immelmann",
                                            "ravine-roll",
                                            "airtime-hill",
                                            "journey",
                                            "terminal-overpass",
                                            "terminal-brake"};
    return names[static_cast<int>(r)];
}
Control controlAt(const Program &p, double time) {
    auto hi = std::upper_bound(p.controls.begin(), p.controls.end(), time,
                               [](double t, const Control &c) { return t < c.time; });
    Control out;
    if (hi == p.controls.begin())
        out = p.controls.front();
    else if (hi == p.controls.end())
        out = p.controls.back();
    else {
        const auto &a = *(hi - 1);
        const auto &b = *hi;
        const double h = b.time - a.time, x = std::clamp((time - a.time) / h, 0., 1.);
        const std::array<double, 4> av{a.normal, a.lateral, a.twist, a.drive},
            bv{b.normal, b.lateral, b.twist, b.drive};
        std::array<double *, 4> dst{&out.normal, &out.lateral, &out.twist, &out.drive};
        out.time = time;
        for (std::size_t i = 0; i < 4; ++i) {
            const auto q = interpolate(av[i], a.first[i], a.second[i], bv[i], b.first[i], b.second[i], h, x);
            *dst[i] = q.v;
            out.first[i] = q.d;
            out.second[i] = q.dd;
        }
    }
    for (const auto &phase : p.twists)
        if (time >= phase.begin && time <= phase.end) {
            // Septic angle: velocity and its first two derivatives vanish at ports.
            const double h = phase.end - phase.begin, x = (time - phase.begin) / h, y = 1 - x, d = 1 - 2 * x,
                         k = 140 * phase.angle;
            out.twist += k * x * x * x * y * y * y / h;
            out.first[2] += 3 * k * x * x * y * y * d / (h * h);
            out.second[2] += 6 * k * x * y * (d * d - x * y) / (h * h * h);
        }
    return out;
}
void validateProgram(const Program &p) {
    if (!p.geometry.empty()) {
        validateSpline(p);
        return;
    }
    if (p.controls.size() < 2 || p.controls.size() > 1000 || p.controls.front().time != 0 ||
        p.duration() > 200 || !finite(p.initial.p) || !finite(p.initial.t) || !finite(p.initial.u) ||
        !(p.initial.v >= 0 && p.initial.v < 150) || p.rolling < 0 || p.drag < 0)
        throw std::runtime_error("Invalid programme");
    if (std::abs(norm(p.initial.t) - 1) > 1e-8 || std::abs(norm(p.initial.u) - 1) > 1e-8 ||
        std::abs(dot(p.initial.t, p.initial.u)) > 1e-8)
        throw std::runtime_error("Nonorthonormal source frame");
    double prior = -1;
    for (const auto &c : p.controls) {
        if (!std::isfinite(c.time) || c.time <= prior)
            throw std::runtime_error("Invalid control times");
        prior = c.time;
        for (double x : {c.normal, c.lateral, c.twist, c.drive})
            if (!std::isfinite(x) || std::abs(x) > 100)
                throw std::runtime_error("Invalid control value");
        for (double x : c.first)
            if (!std::isfinite(x) || std::abs(x) > 1000)
                throw std::runtime_error("Invalid first derivative");
        for (double x : c.second)
            if (!std::isfinite(x) || std::abs(x) > 10000)
                throw std::runtime_error("Invalid second derivative");
    }
    for (const auto &t : p.twists)
        if (!std::isfinite(t.angle) || std::abs(t.angle) > 8 * pi || t.begin < 0 || t.end > p.duration() ||
            t.end <= t.begin)
            throw std::runtime_error("Invalid twist interval");
}
State advance(const State &start, const Program &p, double time, double dt) {
    struct Stage {
        Q q;
        Vec3 dp;
        double dv, ds, loss, drive;
    };
    auto eval = [&](Q q, double v, double at) {
        State s = start;
        s.t = turned(start.t, q);
        s.u = turned(start.u, q);
        s.v = p.role == Role::Terminal ? std::max(0., v) : v;
        auto d = derivative(s, p, at);
        if (p.role == Role::Terminal && s.v == 0 && d.dv < 0)
            d.dv = 0;
        return Stage{{-.5 * dot(d.omega, q.v), (d.omega * q.w + cross(d.omega, q.v)) * .5},
                     s.t * s.v,
                     d.dv,
                     s.v,
                     loss(p, s.v) * s.v,
                     controlAt(p, at).drive * s.v};
    };
    const Q identity;
    const auto a = eval(identity, start.v, time),
               b = eval(identity + a.q * (dt / 2), start.v + a.dv * dt / 2, time + dt / 2),
               c = eval(identity + b.q * (dt / 2), start.v + b.dv * dt / 2, time + dt / 2),
               d = eval(identity + c.q * dt, start.v + c.dv * dt, time + dt);
    const auto rotation = normalized(identity + (a.q + b.q * 2 + c.q * 2 + d.q) * (dt / 6));
    State s = start;
    s.p += (a.dp + b.dp * 2 + c.dp * 2 + d.dp) * (dt / 6);
    s.v += (a.dv + 2 * b.dv + 2 * c.dv + d.dv) * dt / 6;
    if (p.role == Role::Terminal)
        s.v = std::max(0., s.v);
    s.s += (a.ds + 2 * b.ds + 2 * c.ds + d.ds) * dt / 6;
    s.lossWork += (a.loss + 2 * b.loss + 2 * c.loss + d.loss) * dt / 6;
    s.driveWork += (a.drive + 2 * b.drive + 2 * c.drive + d.drive) * dt / 6;
    s.t = unit(turned(start.t, rotation));
    s.u = turned(start.u, rotation);
    s.u = unit(s.u - s.t * dot(s.t, s.u));
    if (!finite(s.p) || norm(s.p) > 100000)
        throw std::runtime_error("Source left site domain");
    return s;
}
Jet jetAt(const State &q, const Program &p, double time) {
    // Analytic differentiation of the physical source ODE, independently of
    // the canonical polynomials. Based on the reviewed archived FVD equations.
    const auto c = controlAt(p, time);
    if (q.v < .1) {
        derivative(q, p, time);
        return {
            {q.p, q.t, Vec3{}, Vec3{}, Vec3{}}, {q.u, Vec3{}, Vec3{}, Vec3{}}, q.s, time, q.v, c.drive, 0};
    }
    const Vec3 t = q.t, u = q.u, r = cross(t, u);
    const double v = q.v, v2 = v * v, along = dot(g, t), vd = c.drive + along - loss(p, v);
    const Vec3 a = g + u * (gravity * c.normal) + r * (gravity * c.lateral), td = (a - t * along) / v;
    const double h = t.x * t.x + t.y * t.y, n = t.x * td.y - t.y * td.x;
    double w = c.twist, wd = c.first[2], wdd = c.second[2];
    if (p.gravityBank)
        w += t.z * n / h;
    const Vec3 omega = cross(t, td) + t * w, ud = cross(omega, u), rd = cross(omega, r);
    const double alongD = dot(g, td),
                 vdd = c.first[3] + alongD -
                       (p.rolling / .2 * (1 - std::pow(std::tanh(v / .2), 2)) + 2 * p.drag * v) * vd;
    const Vec3 ad = (u * c.first[0] + ud * c.normal + r * c.first[1] + rd * c.lateral) * gravity;
    const Vec3 tdd = (ad - td * along - t * alongD - td * vd) / v;
    const double nd = t.x * tdd.y - t.y * tdd.x, hd = 2 * (t.x * td.x + t.y * td.y),
                 yawD = p.gravityBank ? n / h : 0, yawDD = p.gravityBank ? nd / h - n * hd / (h * h) : 0;
    if (p.gravityBank)
        wd += td.z * yawD + t.z * yawDD;
    const Vec3 omegaD = cross(t, tdd) + td * w + t * wd, udd = cross(omegaD, u) + cross(omega, ud),
               rdd = cross(omegaD, r) + cross(omega, rd);
    const Vec3 add = (u * c.second[0] + ud * (2 * c.first[0]) + udd * c.normal + r * c.second[1] +
                      rd * (2 * c.first[1]) + rdd * c.lateral) *
                     gravity;
    const Vec3 tddd =
        (add - tdd * along - td * (2 * alongD) - t * dot(g, tdd) - tdd * (2 * vd) - td * vdd) / v;
    if (p.gravityBank) {
        const double ndd = td.x * tdd.y - td.y * tdd.x + t.x * tddd.y - t.y * tddd.x,
                     hdd = 2 * (td.x * td.x + td.y * td.y + t.x * tdd.x + t.y * tdd.y),
                     yawDDD =
                         ndd / h - 2 * nd * hd / (h * h) - n * hdd / (h * h) + 2 * n * hd * hd / (h * h * h);
        wdd += tdd.z * yawD + 2 * td.z * yawDD + t.z * yawDDD;
    }
    const Vec3 omegaDD = cross(td, tdd) + cross(t, tddd) + tdd * w + td * (2 * wd) + t * wdd,
               uddd = cross(omegaDD, u) + cross(omegaD, ud) * 2 + cross(omega, udd);
    auto second = [&](Vec3 f, Vec3 s) { return s / v2 - f * (vd / (v2 * v)); };
    auto third = [&](Vec3 f, Vec3 s, Vec3 d) {
        return d / (v2 * v) - s * (3 * vd / (v2 * v2)) + f * (3 * vd * vd / (v2 * v2 * v) - vdd / (v2 * v2));
    };
    return {{q.p, t, td / v, second(td, tdd), third(td, tdd, tddd)},
            {u, ud / v, second(ud, udd), third(ud, udd, uddd)},
            q.s,
            time,
            q.v,
            c.drive,
            0};
}
Shot shoot(const Program &p, double step, const Cancel &cancel) {
    if (!p.geometry.empty())
        return shootSpline(p, step * 50, cancel);
    validateProgram(p);
    Shot out;
    out.end = p.initial;
    out.maximumHeight = out.minimumHeight = p.initial.p.z;
    out.yaw = std::atan2(p.initial.t.y, p.initial.t.x);
    out.pitchTurn = pitch(p.initial.t);
    for (std::size_t i = 1; i < p.controls.size(); ++i) {
        const double begin = p.controls[i - 1].time, h = p.controls[i].time - begin;
        const int count = std::max(1, int(std::ceil(h / step)));
        const double dt = h / count;
        for (int j = 0; j < count; ++j) {
            if ((j & 63) == 0)
                poll(cancel);
            out.end = advance(out.end, p, begin + j * dt, dt);
            out.maximumHeight = std::max(out.maximumHeight, out.end.p.z);
            out.minimumHeight = std::min(out.minimumHeight, out.end.p.z);
            out.yaw += std::remainder(std::atan2(out.end.t.y, out.end.t.x) - out.yaw, 2 * pi);
            out.pitchTurn += std::remainder(std::atan2(out.end.t.z, out.end.t.x) - out.pitchTurn, 2 * pi);
        }
    }
    return out;
}
std::vector<Jet> replay(const Program &p, double step, const Cancel &cancel) {
    if (!p.geometry.empty())
        return replaySpline(p, step * 50, cancel);
    validateProgram(p);
    std::vector<Jet> result;
    result.reserve(std::size_t(p.duration() / step) + p.controls.size() + 1);
    State state = p.initial;
    result.push_back(jetAt(state, p, 0));
    for (std::size_t i = 1; i < p.controls.size(); ++i) {
        const double begin = p.controls[i - 1].time, h = p.controls[i].time - begin;
        const int count = std::max(1, int(std::ceil(h / step)));
        const double dt = h / count;
        for (int j = 0; j < count; ++j) {
            if ((j & 63) == 0)
                poll(cancel);
            state = advance(state, p, begin + j * dt, dt);
            result.push_back(jetAt(state, p, begin + (j + 1) * dt));
        }
    }
    return result;
}
Program hill(const State &state, const HillShape &h, const Cancel &cancel) {
    Program r;
    r.initial = state;
    r.gravityBank = h.bankDegrees != 0;
    const double base = state.p.z;
    auto controls = [&](const std::array<double, 3> &p) {
        const double peak = h.riseRamp + p[0], air = peak + h.releaseRamp, airEnd = air + p[1],
                     pull = airEnd + h.recoveryRamp, release = pull + p[2];
        r.controls = {{0, 1},
                      {h.riseRamp, h.positive},
                      {peak, h.positive},
                      {air, h.negative},
                      {airEnd, h.descentNegative},
                      {pull, h.exitPositive},
                      {release, h.exitPositive},
                      {release + h.exitRelease, 1}};
        if (h.crownRelief > 0) {
            const double center = air + (airEnd - air) * h.crownFraction, half = h.crownHold / 2;
            r.controls.insert(r.controls.begin() + 4,
                              {{center - half - h.crownTransition, h.negative},
                               {center - half, h.crownRelief},
                               {center + half, h.crownRelief},
                               {center + half + h.crownTransition, h.descentNegative}});
        }
        r.twists.clear();
        if (h.bankDegrees != 0) {
            r.twists = {{air, airEnd, rad(h.bankDegrees)},
                        {pull, release + h.exitRelease, -rad(h.bankDegrees)}};
        }
    };
    auto residual = [&](const std::array<double, 3> &p) {
        controls(p);
        const auto q = shoot(r, .025, cancel);
        return std::array<double, 3>{(q.maximumHeight - base - h.height) / 100,
                                     (q.end.p.z - base - h.exitHeight) / 100, pitch(q.end.t)};
    };
    std::array<double, 3> selected{};
    bool found = false;
    for (auto p : std::array<std::array<double, 3>, 4>{
             {{.8, 4.5, .8}, {1.5, 5.5, .5}, {.1, 3.5, 1.2}, {2., 6., 1.}}}) {
        try {
            if (solve<3>(p, {{{.001, 7}, {.1, 13}, {.001, 7}}}, residual)) {
                selected = p;
                found = true;
                break;
            }
        } catch (const Cancelled &) {
            throw;
        } catch (const std::runtime_error &) {
        }
    }
    if (!found)
        throw std::runtime_error("Hill cannot reach authored height and live exit at available energy");
    controls(selected);
    return r;
}
Program dive(const State &state, double drop, double exitPitch, const Cancel &cancel, double crestNormal) {
    Program r;
    r.initial = state;
    constexpr double maxPitch = 88 * pi / 180;
    auto controls = [&](const std::array<double, 3> &p) {
        const double crest = .7 + p[0], vertical = crest + .7, fall = vertical + p[1], load = fall + 1.7,
                     release = load + p[2];
        r.controls = {{0, 1},
                      {.7, 0},
                      {crest, crestNormal},
                      {vertical, std::cos(maxPitch)},
                      {fall, std::cos(maxPitch)},
                      {load, 3.2},
                      {release, 3.2},
                      {release + 1.2, std::cos(exitPitch)}};
    };
    auto residual = [&](const std::array<double, 3> &p) {
        controls(p);
        auto partial = r;
        partial.controls.resize(4);
        const auto v = shoot(partial, .02, cancel), q = shoot(r, .02, cancel);
        return std::array<double, 3>{pitch(v.end.t) + maxPitch, (q.end.p.z - state.p.z + drop) / 100,
                                     pitch(q.end.t) - exitPitch};
    };
    std::ostringstream diagnostic;
    diagnostic << " v=" << state.v << " drop=" << drop << " pitch=" << pitch(state.t);
    std::array<double, 3> selected{};
    bool found = false;
    for (auto p :
         std::array<std::array<double, 3>, 4>{{{1, 1, 1}, {.3, 2, 1.5}, {1.5, .5, .5}, {.7, 3, 1}}}) {
        try {
            if (solve<3>(p, {{{.001, 8}, {.02, 8}, {.001, 8}}}, residual)) {
                selected = p;
                found = true;
                break;
            }
            const auto e = residual(p);
            diagnostic << " [" << p[0] << "," << p[1] << "," << p[2] << " => " << e[0] << "," << e[1] << ","
                       << e[2] << "]";
        } catch (const Cancelled &) {
            throw;
        } catch (const std::runtime_error &e) {
            diagnostic << " [" << e.what() << "]";
        }
    }
    if (!found)
        throw std::runtime_error("Cliff cannot reach authored drop and inclined exit" + diagnostic.str());
    controls(selected);
    return r;
}
Program loop(const State &state, double height, double yaw, const Cancel &cancel) {
    Program r;
    r.initial = state;
    const double heading = std::atan2(state.t.y, state.t.x);
    constexpr double normal = 3.7;
    auto controls = [&](const std::array<double, 6> &p) {
        const double peak = 1.2 + p[0], apex = peak + p[1], crest = apex + .7, load = crest + 1.2,
                     release = load + p[2];
        r.controls = {{0, 1},      {1.2, normal, p[3]}, {peak, normal, p[3]}, {apex, .9},
                      {crest, .9}, {load, p[5]},        {release, p[5]},      {release + 1, 1}};
        r.twists = {{crest, release + 1, p[4]}};
    };
    auto residual = [&](const std::array<double, 6> &p) {
        controls(p);
        auto partial = r;
        partial.twists.clear();
        partial.controls.resize(4);
        const auto a = shoot(partial, .025, cancel).end, q = shoot(r, .025, cancel).end;
        const Vec3 right{std::sin(heading + yaw), -std::cos(heading + yaw), 0};
        return std::array<double, 6>{(a.p.z - state.p.z - height) / 100,
                                     a.t.z,
                                     q.t.z,
                                     std::remainder(std::atan2(q.t.y, q.t.x) - heading - yaw, 2 * pi),
                                     dot(q.u, right),
                                     (q.p.z - state.p.z) / 100};
    };
    std::array<double, 6> selected{};
    bool found = false;
    for (auto p : std::array<std::array<double, 6>, 4>{{{2, 1.8, 2.3, -2 * yaw, 0, normal},
                                                        {2.5, 1, 2.5, -2 * yaw, .1, normal},
                                                        {1.5, 2.5, 2, -2 * yaw, -.1, normal},
                                                        {3, .7, 3, -2 * yaw, 0, normal}}}) {
        try {
            if (solve<6>(p, {{{.001, 6}, {.6, 4}, {.001, 7}, {-1.2, 1.2}, {-.9, .9}, {2.5, 4.3}}},
                         residual)) {
                controls(p);
                auto partial = r;
                partial.twists.clear();
                partial.controls.resize(4);
                if (shoot(partial, .02, cancel).end.u.z < -.5 && shoot(r, .02, cancel).end.u.z > .9) {
                    selected = p;
                    found = true;
                    break;
                }
            }
        } catch (const Cancelled &) {
            throw;
        } catch (const std::runtime_error &) {
        }
    }
    if (!found)
        throw std::runtime_error("Loop cannot reach authored apex and yaw with live energy");
    controls(selected);
    return r;
}
Program immelmann(const State &state, double height, double exitHeight, const Cancel &cancel) {
    Program r;
    r.initial = state;
    constexpr double ramp = 1.4, normal = 4.;
    auto controls = [&](const std::array<double, 5> &p) {
        const double apex = ramp + p[0] + p[1], roll = apex + p[2], pull = roll + ramp;
        r.controls = {{0, 1},     {ramp, normal}, {ramp + p[0], normal}, {apex, .6},
                      {roll, .3}, {pull, normal}, {pull + p[4], 1}};
        r.twists = {{apex - .35 * p[1], roll, p[3]}};
    };
    auto residual = [&](const std::array<double, 5> &p) {
        controls(p);
        auto a = r;
        a.twists.clear();
        a.controls.resize(4);
        auto b = r;
        b.controls.resize(5);
        const auto apex = shoot(a, .025, cancel).end, roll = shoot(b, .025, cancel).end,
                   q = shoot(r, .025, cancel).end;
        const Vec3 right = unit(cross(roll.t, Vec3{0, 0, 1}));
        return std::array<double, 5>{(apex.p.z - state.p.z - height) / 100, apex.t.z,
                                     std::atan2(dot(roll.u, right), roll.u.z),
                                     (q.p.z - state.p.z - exitHeight) / 50, q.t.z};
    };
    std::array<double, 5> selected{};
    bool found = false;
    for (auto p : std::array<std::array<double, 5>, 3>{
             {{1.2, 2, 3, pi, 1}, {.5, 3.2, 3, pi, 1}, {2, 1, 3.5, pi, 1}}}) {
        for (std::size_t i : {0U, 1U, 2U, 4U})
            p[i] *= state.v / 53;
        try {
            if (solve<5>(p, {{{.001, 8}, {.2, 12}, {.5, 8}, {1, 5.2}, {.6, 6}}}, residual)) {
                selected = p;
                found = true;
                break;
            }
        } catch (const Cancelled &) {
            throw;
        } catch (const std::runtime_error &) {
        }
    }
    if (!found)
        throw std::runtime_error("Immelmann cannot reach authored height and descending roll exit");
    controls(selected);
    return r;
}
} // namespace coaster
