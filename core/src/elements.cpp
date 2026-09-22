#include "coaster/elements.hpp"
#include <sstream>

namespace coaster {
namespace {
std::array<double, 5> step9(double q, double length) {
    if (q <= 0)
        return {};
    if (q >= length)
        return {1, 0, 0, 0, 0};
    constexpr std::array<double, 10> c{0, 0, 0, 0, 0, 126, -420, 540, -315, 70};
    const double u = q / length;
    std::array<double, 5> out{};
    double scale = 1;
    for (int d = 0; d < 5; ++d) {
        for (int i = 9; i >= d; --i) {
            double factor = 1;
            for (int j = 0; j < d; ++j)
                factor *= i - j;
            out[std::size_t(d)] = out[std::size_t(d)] * u + c[std::size_t(i)] * factor;
        }
        out[std::size_t(d)] /= scale;
        scale *= length;
    }
    return out;
}
} // namespace
Program ascent(const State &state, double rise, double run, double exitSpeed, const Cancel &cancel) {
    Program p;
    p.initial = state;
    const double yaw = std::atan2(state.t.y, state.t.x);
    const int count = int(std::ceil(run / 2));
    double distance = 0;
    auto slope = [&](double q) { return rise * step9(q, run)[1]; };
    auto w = [&](double q) { return std::hypot(1., slope(q)); };
    for (int i = 0; i <= count; ++i) {
        const double q = run * i / count, previous = run * (i - 1) / count;
        if (i)
            distance += (q - previous) * (w(previous) + 4 * w((q + previous) / 2) + w(q)) / 6;
        const auto h = step9(q, run);
        const double a = rise * h[1], b = rise * h[2], c = rise * h[3], d = rise * h[4],
                     v = std::hypot(1., a);
        GeometryControl k;
        k.s = distance;
        k.value = {std::atan(a), yaw, 0, 0};
        k.first[0] = b / std::pow(v, 3);
        k.second[0] = c / std::pow(v, 4) - 3 * a * b * b / std::pow(v, 6);
        k.third[0] = d / std::pow(v, 5) - (10 * a * b * c + 3 * b * b * b) / std::pow(v, 7) +
                     18 * a * a * b * b * b / std::pow(v, 9);
        const auto start = step9(q, 120), end = step9(q - (run - 120), 120);
        const double f = start[0] * (1 - end[0]), fq = start[1] * (1 - end[0]) - start[0] * end[1];
        const double fqq = start[2] * (1 - end[0]) - 2 * start[1] * end[1] - start[0] * end[2];
        const double fqqq =
            start[3] * (1 - end[0]) - 3 * start[2] * end[1] - 3 * start[1] * end[2] - start[0] * end[3];
        const double vq = a * b / v, vqq = (b * b + a * c) / v - a * a * b * b / std::pow(v, 3);
        k.value[3] = f;
        k.first[3] = fq / v;
        k.second[3] = fqq / (v * v) - fq * vq / std::pow(v, 3);
        k.third[3] = fqqq / std::pow(v, 3) - (3 * fqq * vq + fq * vqq) / std::pow(v, 4) +
                     3 * fq * vq * vq / std::pow(v, 5);
        p.geometry.push_back(k);
    }
    const auto base = p.geometry;
    auto set = [&](double motor) {
        p.geometry = base;
        for (auto &k : p.geometry) {
            k.value[3] *= motor;
            k.first[3] *= motor;
            k.second[3] *= motor;
            k.third[3] *= motor;
        }
    };
    set(8);
    const double v8 = shootSpline(p, 1, cancel).end.v;
    set(9);
    const double v9 = shootSpline(p, 1, cancel).end.v;
    const double motor = 8 + (exitSpeed * exitSpeed - v8 * v8) / (v9 * v9 - v8 * v8);
    if (!(motor > 0 && motor < 15))
        throw std::runtime_error("Ascent outside motor domain");
    set(motor);
    p.geometricDuration = replaySpline(p, 1, cancel).back().time;
    return p;
}
Program wave(const State &state, double height, const Cancel &cancel) {
    Program r;
    r.initial = state;
    r.gravityBank = true;
    const double startYaw = std::atan2(state.t.y, state.t.x);
    auto controls = [&](const std::array<double, 4> &p) {
        const double peak = .75 + p[0], low = peak + 1.5, fall = low + p[1], loaded = fall + 1.5,
                     release = loaded + p[3], end = release + 1;
        r.controls = {{0, 1},      {.75, 3.9},    {peak, 3.9},    {low, 1.3},
                      {fall, 1.3}, {loaded, 3.6}, {release, 3.6}, {end, 1}};
        r.twists = {{.15, 1.85, -rad(70)}, {1.85 + p[2] * (end - 3.05), end, rad(70)}};
    };
    auto residual = [&](const std::array<double, 4> &p) {
        controls(p);
        const auto q = shoot(r, .025, cancel);
        return std::array<double, 4>{(q.maximumHeight - state.p.z - height) / 100,
                                     (q.end.p.z - state.p.z) / 100, std::asin(q.end.t.z),
                                     q.yaw - startYaw - pi};
    };
    std::array<double, 4> selected{};
    bool found = false;
    std::ostringstream diagnostic;
    for (auto p : std::array<std::array<double, 4>, 6>{{{.66, 7.95, .98, 1.02},
                                                        {.5, 9, .95, .8},
                                                        {1, 7, .8, .3},
                                                        {.1, 3, .8, .5},
                                                        {.3, 4, .5, 1.5},
                                                        {.1, 6, 1, 1}}}) {
        try {
            if (solve<4>(p, {{{.001, 6}, {1, 14}, {0, 1}, {.1, 8}}}, residual)) {
                selected = p;
                found = true;
                break;
            }
            const auto e = residual(p);
            diagnostic << " [" << p[0] << "," << p[1] << "," << p[2] << "," << p[3] << " => " << e[0] << ","
                       << e[1] << "," << e[2] << "," << e[3] << "]";
        } catch (const Cancelled &) {
            throw;
        } catch (const std::runtime_error &) {
        }
    }
    if (!found)
        throw std::runtime_error("Rising wave cannot meet height, 180-degree turn and live exit" +
                                 diagnostic.str());
    controls(selected);
    return r;
}
Program edgeAct(const State &state, double headingChange, double negative, const Cancel &cancel) {
    Program r;
    r.initial = state;
    r.gravityBank = true;
    const double sign = std::copysign(1., headingChange), yaw = std::atan2(state.t.y, state.t.x),
                 inbank = rad(45) * std::min(1., state.v / 47);
    auto controls = [&](const std::array<double, 3> &p) {
        const double t1 = 1, t2 = t1 + p[0], t3 = t2 + 1, t4 = t3 + 1, t5 = t4 + 1.1, t6 = t5 + p[1],
                     t7 = t6 + 1.1;
        r.controls = {{0, 1},         {t1, 2.3}, {t2, 2.3}, {t3, negative},
                      {t4, negative}, {t5, 2.3}, {t6, 2.3}, {t7, 1}};
        r.twists = {{.1, t2, -sign * inbank},
                    {t2, t4, sign * (inbank + rad(25))},
                    {t4, t6, -sign * (rad(25) + p[2])},
                    {t6, t7, sign * p[2]}};
    };
    auto residual = [&](const std::array<double, 3> &p) {
        controls(p);
        const auto q = shoot(r, .025, cancel);
        return std::array<double, 3>{(q.end.p.z - state.p.z) / 50, std::asin(q.end.t.z),
                                     q.yaw - yaw - headingChange};
    };
    bool found = false;
    std::array<double, 3> selected{};
    for (auto p : std::array<std::array<double, 3>, 4>{
             {{1.1, 2, rad(45)}, {.5, 2.5, rad(35)}, {2, 2, rad(60)}, {.3, 3, rad(30)}}}) {
        try {
            if (solve<3>(p, {{{.01, 4}, {.2, 7}, {rad(5), rad(78)}}}, residual)) {
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
        throw std::runtime_error("Inbank/outbank/inbank act cannot reach its live exit at " +
                                 std::to_string(state.v) + " m/s");
    controls(selected);
    return r;
}
Program brake(const State &state, double targetSpeed, double seconds) {
    Program p;
    p.initial = state;
    if (targetSpeed == 0)
        p.role = Role::Terminal;
    double peak = (targetSpeed - state.v) / (seconds - .5);
    auto controls = [&](double a) {
        p.controls = {{0, 1, 0, 0, 0}, {.5, 1, 0, 0, a}, {seconds - .5, 1, 0, 0, a}, {seconds, 1, 0, 0, 0}};
    };
    if (targetSpeed == 0) {
        double low = 0, high = 25;
        for (int i = 0; i < 45; ++i) {
            peak = -(low + high) / 2;
            controls(peak);
            const auto q = shoot(p, .005);
            if (q.end.v > 1e-9)
                low = -peak;
            else
                high = -peak;
        }
        peak = -high;
    } else
        for (int i = 0; i < 6; ++i) {
            controls(peak);
            const auto q = shoot(p, .01);
            peak += (targetSpeed - q.end.v) / (seconds - .5);
        }
    controls(peak);
    return p;
}
Program inclinedBoost(const State &state, double targetSpeed, const Cancel &cancel) {
    Program r;
    r.initial = state;
    auto controls = [&](const std::array<double, 2> &p) {
        const double end = 1.6 + p[0];
        r.controls = {
            {0, state.u.z, 0, 0, 0}, {.8, 3.0, 0, 0, p[1]}, {.8 + p[0], 3.0, 0, 0, p[1]}, {end, 1, 0, 0, 0}};
    };
    auto residual = [&](const std::array<double, 2> &p) {
        controls(p);
        const auto q = shoot(r, .015, cancel);
        return std::array<double, 2>{std::asin(q.end.t.z), (q.end.v - targetSpeed) / 50};
    };
    std::array<double, 2> p{.8, 15};
    if (!solve<2>(p, {{{.001, 8}, {1, 40}}}, residual))
        throw std::runtime_error("Inclined LSM cannot meet speed and low recovery");
    controls(p);
    return r;
}
} // namespace coaster
