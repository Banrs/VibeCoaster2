#include "coaster/journey.hpp"

namespace coaster {
namespace {
constexpr double crestHeight = 32, crestWidth = 500, tailLength = 140;
constexpr double crestOffset = crestWidth / 2 - tailLength;
double S(double u) {
    u = std::clamp(u, 0., 1.);
    return std::pow(u, 5) * (126 + u * (-420 + u * (540 + u * (-315 + 70 * u))));
}
double Sd(double u) {
    u = std::clamp(u, 0., 1.);
    return 630 * std::pow(u * (1 - u), 4);
}
double I(double x) {
    return std::pow(x, 6) * (21 + x * (-60 + x * (67.5 + x * (-35 + 7 * x))));
}
double progress(double u) {
    constexpr double e = .28;
    if (u < e)
        return e * I(u / e) / (1 - e);
    if (u <= 1 - e)
        return (u - e / 2) / (1 - e);
    const double x = (u - (1 - e)) / e;
    return (1 - 1.5 * e + e * (x - I(x))) / (1 - e);
}
double progressD(double u) {
    constexpr double e = .28;
    return (u < e ? S(u / e) : u > 1 - e ? 1 - S((u - (1 - e)) / e) : 1) / (1 - e);
}
double hill(double u) {
    return u > 0 && u < 1 ? 1024 * std::pow(u * (1 - u), 5) : 0;
}
double hillD(double u) {
    return u > 0 && u < 1 ? 5120 * std::pow(u * (1 - u), 4) * (1 - 2 * u) : 0;
}
double falling(int n, int d) {
    double value = 1;
    for (int i = 0; i < d; ++i)
        value *= n - i;
    return value;
}
double hillDerivative(double u, int derivative) {
    constexpr std::array<double, 6> coefficients{1, -5, 10, -10, 5, -1};
    double value = 0;
    for (int i = 0; i < 6; ++i)
        value += 1024 * coefficients[std::size_t(i)] * falling(i + 5, derivative) *
                 std::pow(u, i + 5 - derivative);
    return value;
}
struct HeightCurve {
    std::array<double, 10> coefficients{};
    double length{};
    HeightCurve(double distance, const std::array<double, 5> &first, const std::array<double, 5> &last)
        : length(distance) {
        constexpr std::array<double, 5> factorial{1, 1, 2, 6, 24};
        for (std::size_t i = 0; i < 5; ++i)
            coefficients[i] = first[i] * std::pow(length, int(i)) / factorial[i];
        std::array<std::array<double, 6>, 5> matrix{};
        for (int i = 0; i < 5; ++i) {
            double rhs = last[std::size_t(i)] * std::pow(length, i);
            for (int j = i; j < 5; ++j)
                rhs -= coefficients[std::size_t(j)] * falling(j, i);
            for (int j = 0; j < 5; ++j)
                matrix[std::size_t(i)][std::size_t(j)] = falling(j + 5, i);
            matrix[std::size_t(i)][5] = rhs;
        }
        if (!eliminate<5>(matrix))
            throw std::runtime_error("Cannot interpolate braking height jets");
        for (std::size_t i = 0; i < 5; ++i)
            coefficients[i + 5] = matrix[i][5];
    }
    double slope(double distance) const {
        double value = 0, u = distance / length;
        for (int i = 9; i >= 1; --i)
            value = value * u + i * coefficients[std::size_t(i)];
        return value / length;
    }
};
template <class F> void controls(Program &p, double length, F value, const std::vector<double> &extra = {}) {
    const int count = std::max(30, int(std::ceil(length / 4)));
    std::vector<double> positions;
    for (int i = 0; i <= count; ++i)
        positions.push_back(length * i / count);
    for (double at : extra)
        if (at > 0 && at < length)
            positions.push_back(at);
    std::sort(positions.begin(), positions.end());
    positions.erase(std::unique(positions.begin(), positions.end()), positions.end());
    constexpr double h = .05;
    for (double s : positions) {
        GeometryControl k;
        k.s = s;
        const auto v = value(s), a = value(s - h), b = value(s + h), aa = value(s - 2 * h),
                   bb = value(s + 2 * h);
        k.value = v;
        for (std::size_t j = 0; j < 4; ++j) {
            k.first[j] = (b[j] - a[j]) / (2 * h);
            k.second[j] = (b[j] - 2 * v[j] + a[j]) / (h * h);
            k.third[j] = (bb[j] - 2 * b[j] + 2 * a[j] - aa[j]) / (2 * h * h * h);
        }
        p.geometry.push_back(k);
    }
}
} // namespace
double coastingExitSpeed(double speed, double seconds, double rolling, double drag) {
    const double q = std::sqrt(rolling / drag),
                 angle = std::atan(speed / q) - std::sqrt(rolling * drag) * seconds;
    if (angle <= 0)
        throw std::runtime_error("Journey exceeds available coasting energy");
    return q * std::tan(angle);
}
Program terminalEntry(const State &state, const Cancel &cancel, const TerminalShape &shape) {
    Program p;
    p.initial = state;
    p.role = Role::TerminalOverpass;
    const double yaw = std::atan2(state.t.y, state.t.x);
    // Preserve the crest's C4 height jet, ease to a shallow crossing grade,
    // then descend after clearing the valley track. Terrain stays unchanged.
    constexpr double flatten = 30, descentStart = 95, shallowGrade = -.008;
    const double shoulderHeight = shape.shoulderHeight;
    std::array<double, 5> inherited{};
    const double u = (crestWidth - tailLength) / crestWidth;
    for (int i = 0; i < 5; ++i)
        inherited[std::size_t(i)] = crestHeight * hillDerivative(u, i) / std::pow(crestWidth, i);
    const HeightCurve entry(flatten, inherited, {shoulderHeight, shallowGrade, 0, 0, 0});
    const double descentHeight = shoulderHeight + shallowGrade * (descentStart - flatten);
    const HeightCurve descent(tailLength - descentStart, {descentHeight, shallowGrade, 0, 0, 0},
                              {0, 0, 0, 0, 0});
    auto value = [&](double s) {
        const double slope = s < flatten        ? entry.slope(s)
                             : s < descentStart ? shallowGrade
                                                : descent.slope(s - descentStart);
        const double drive = -shape.deceleration * S(s / 15) * (1 - S((s - (tailLength - 15)) / 15));
        return std::array<double, 4>{std::asin(slope), yaw, 0, drive};
    };
    controls(p, tailLength, value, {flatten, descentStart});
    p.geometricDuration = replaySpline(p, .5, cancel).back().time;
    return p;
}
BrakingParts finishBrakes(const State &state, double seconds, const Cancel &cancel, double finalEntrySpeed,
                          const TerminalShape &shape) {
    BrakingParts result;
    result.entry = terminalEntry(state, cancel, shape);
    State stopping = shoot(result.entry, .01, cancel).end;
    if (finalEntrySpeed > 0)
        stopping.v = finalEntrySpeed;
    const double remaining = seconds - result.entry.duration();
    if (remaining < 1)
        throw std::runtime_error("Brake duration cannot include the crossing descent");
    result.stop = brake(stopping, 0, remaining);
    return result;
}
BrakePlan planBrakes(double speed, double seconds, const Cancel &cancel, double finalEntrySpeed,
                     const TerminalShape &shape) {
    const double u = (crestWidth - tailLength) / crestWidth;
    const double height = crestHeight * hill(u), pitch = std::asin(crestHeight * hillD(u) / crestWidth);
    State state;
    state.p = {0, 0, height};
    state.t = {std::cos(pitch), 0, std::sin(pitch)};
    state.u = {-std::sin(pitch), 0, std::cos(pitch)};
    state.v = speed;
    const auto parts = finishBrakes(state, seconds, cancel, finalEntrySpeed, shape);
    const auto end = shoot(parts.stop, .005, cancel).end;
    return {end.p.x, height, pitch, parts.entry.duration(), parts.stop.duration()};
}
JourneyParts closeJourney(const State &state, Vec3 target, Vec3 exitForward, double seconds,
                          const Cancel &cancel, const std::function<bool(const Program &)> &routeFilter,
                          JourneyHint *hint) {
    if (seconds < 4 || seconds > 60)
        throw std::runtime_error("Journey duration outside authored domain");
    Program p;
    p.initial = state;
    const double endSpeed = coastingExitSpeed(state.v, seconds, p.rolling, p.drag);
    const double length =
        std::log((p.rolling + p.drag * state.v * state.v) / (p.rolling + p.drag * endSpeed * endSpeed)) /
        (2 * p.drag);
    if (length < 450)
        throw std::runtime_error("Insufficient active distance for the final crossing crest");
    const double initial = std::atan2(state.t.y, state.t.x), last = std::atan2(exitForward.y, exitForward.x);
    const double endHeight = crestHeight * hill((crestWidth - tailLength) / crestWidth);
    const double dx = target.x - state.p.x, dy = target.y - state.p.y, dz = target.z - state.p.z - endHeight;
    auto hillPhase = [&](double s) { return (s - (length - crestOffset) + crestWidth / 2) / crestWidth; };
    const double riseWidth = 300, floatWidth = 235, workLength = length - crestOffset - crestWidth / 2,
                 gap = (workLength - riseWidth - floatWidth) / 3;
    if (gap < 20)
        throw std::runtime_error("Finale requires space between its height elements");
    const double riseCenter = gap + riseWidth / 2, floatCenter = 2 * gap + riseWidth + floatWidth / 2;
    auto height = [&](double s) {
        return dz * S(s / length) + crestHeight * hill(hillPhase(s)) +
               18 * hill((s - riseCenter) / riseWidth + .5) + 16 * hill((s - floatCenter) / floatWidth + .5);
    };
    auto pitch = [&](double s) {
        return std::asin(dz * Sd(s / length) / length + crestHeight * hillD(hillPhase(s)) / crestWidth +
                         18 * hillD((s - riseCenter) / riseWidth + .5) / riseWidth +
                         16 * hillD((s - floatCenter) / floatWidth + .5) / floatWidth);
    };
    std::vector<double> cuts{0, 1. / 3, 2. / 3, 1}, bestCuts = cuts;
    double fixedMiddle = 0, bestMiddle = 0;
    auto angles = [&](double turn, const std::array<double, 2> &q) {
        return cuts.size() == 4 ? std::array<double, 5>{0, q[0], q[1], turn, turn}
                                : std::array<double, 5>{0, q[0], fixedMiddle, q[1], turn};
    };
    auto legAt = [&](double u) {
        return std::min(cuts.size() - 2,
                        std::size_t(std::upper_bound(cuts.begin(), cuts.end(), u) - cuts.begin() - 1));
    };
    auto heading = [&](double u, double turn, const std::array<double, 2> &q) {
        u = std::clamp(u, 0., 1.);
        const auto leg = legAt(u);
        const auto a = angles(turn, q);
        return initial + a[leg] +
               (a[leg + 1] - a[leg]) * progress((u - cuts[leg]) / (cuts[leg + 1] - cuts[leg]));
    };
    auto headingD = [&](double u, double turn, const std::array<double, 2> &q) {
        u = std::clamp(u, 0., 1.);
        const auto leg = legAt(u);
        const auto a = angles(turn, q);
        return (a[leg + 1] - a[leg]) * progressD((u - cuts[leg]) / (cuts[leg + 1] - cuts[leg])) /
               ((cuts[leg + 1] - cuts[leg]) * length);
    };
    double bestScore = 1e30, bestTurn = 0, minimumPeak = 1e30;
    int roots = 0, bestWinding = 0;
    double bestOffset = 0;
    std::array<double, 2> best{};
    bool found = false;
    constexpr int integrationCount = 800;
    std::array<double, integrationCount> horizontal{};
    for (int i = 0; i < integrationCount; ++i)
        horizontal[i] = std::cos(pitch(length * (i + .5) / integrationCount)) * length / integrationCount;
    auto search = [&](const std::vector<double> &shape, int winding, double offset,
                      const std::vector<std::array<double, 2>> &seeds) {
        cuts = shape;
        const double turn = std::remainder(last - initial, 2 * pi) + winding * 2 * pi;
        fixedMiddle = turn * .5 + offset;
        // Heading is affine in the two shooting parameters. Cache these
        // weights instead of rebuilding smooth polynomials at every Newton step.
        std::array<std::array<double, 3>, integrationCount> weights;
        for (int i = 0; i < integrationCount; ++i) {
            const double u = (i + .5) / integrationCount, a = heading(u, turn, {0, 0});
            weights[i] = {a, heading(u, turn, {1, 0}) - a, heading(u, turn, {0, 1}) - a};
        }
        auto residual = [&](const std::array<double, 2> &q) {
            double x = 0, y = 0;
            for (int i = 0; i < integrationCount; ++i) {
                const auto &w = weights[i];
                const double a = w[0] + w[1] * q[0] + w[2] * q[1];
                x += std::cos(a) * horizontal[i];
                y += std::sin(a) * horizontal[i];
            }
            return std::array<double, 2>{(x - dx) / 100, (y - dy) / 100};
        };
        std::vector<std::array<double, 2>> solutions;
        for (auto q : seeds) {
            poll(cancel);
            if (!solve<2>(q, {{{-9.4, 9.4}, {-9.4, 9.4}}}, residual))
                continue;
            if (std::any_of(solutions.begin(), solutions.end(), [&](const auto &prior) {
                    return std::hypot(q[0] - prior[0], q[1] - prior[1]) < 1e-4;
                }))
                continue;
            solutions.push_back(q);
            ++roots;
            double maximumG = 0, score = 0;
            for (int i = 0; i <= 300; ++i) {
                const double u = i / 300., at = u * length;
                const double v2 = (state.v * state.v + p.rolling / p.drag) * std::exp(-2 * p.drag * at) -
                                  p.rolling / p.drag - 2 * gravity * height(at);
                const double k = headingD(u, turn, q);
                maximumG = std::max(maximumG, std::hypot(1., v2 * k / gravity));
                score += k * k;
            }
            minimumPeak = std::min(minimumPeak, maximumG);
            if (maximumG > 4.1)
                continue;
            score += .001 * std::abs(winding) + .0001 * (q[0] * q[0] + q[1] * q[1]);
            if (score >= bestScore)
                continue;
            if (routeFilter) {
                Program trial = p;
                trial.id = "return-candidate";
                auto values = [&](double at) {
                    const double u = std::clamp(at / length, 0., 1.), yaw = heading(u, turn, q),
                                 yawS = headingD(u, turn, q);
                    const double v2 = (state.v * state.v + p.rolling / p.drag) * std::exp(-2 * p.drag * at) -
                                      p.rolling / p.drag - 2 * gravity * height(at);
                    return std::array<double, 4>{pitch(at), yaw, -std::atan2(v2 * yawS, gravity), 0};
                };
                controls(trial, length, values);
                trial.geometricDuration = replaySpline(trial, 1, cancel).back().time;
                if (!routeFilter(trial))
                    continue;
            }
            found = true;
            best = q;
            bestTurn = turn;
            bestScore = score;
            bestCuts = cuts;
            bestMiddle = fixedMiddle;
            bestWinding = winding;
            bestOffset = offset;
        }
    };
    // Calibration perturbs physical entry speed and timing. Re-solve and
    // independently check the prior route family before searching new ones.
    if (hint && !hint->cuts.empty())
        search(hint->cuts, hint->winding, hint->middleOffset, {hint->headings});
    if (!found) {
        const std::vector<std::array<double, 2>> seeds{{2, 4},   {-2, -4}, {3, 2},  {-3, -2}, {4, 3},
                                                       {-4, -3}, {2, -2},  {-2, 2}, {4, 5}};
        const std::vector<std::vector<double>> families{
            {0, 1. / 3, 2. / 3, 1}, {0, .22, .65, 1},    {0, .38, .73, 1}, {0, .45, .8, 1},
            {0, .2, .55, 1},        {0, .5, .7, 1},      {0, .25, .5, 1},  {0, .2, .45, .75, 1},
            {0, .25, .5, .75, 1},   {0, .15, .45, .8, 1}};
        for (const auto &shape : families) {
            for (int winding : {0, -1, 1})
                for (double offset : {0., -.6, .6, -1.2, 1.2, -2., 2.}) {
                    if (shape.size() == 4 && offset != 0)
                        continue;
                    search(shape, winding, offset, seeds);
                }
            if (found)
                break;
        }
    }
    cuts = bestCuts;
    fixedMiddle = bestMiddle;
    if (found && hint)
        *hint = {bestCuts, best, bestWinding, bestOffset};
    if (!found)
        throw std::runtime_error("Park journey cannot reach station: seconds=" + std::to_string(seconds) +
                                 " length=" + std::to_string(length) + " roots=" + std::to_string(roots) +
                                 " minimumPeak=" + std::to_string(minimumPeak));
    auto value = [&](double s) {
        const double u = std::clamp(s / length, 0., 1.), yaw = heading(u, bestTurn, best),
                     yawS = headingD(u, bestTurn, best);
        const double v2 = (state.v * state.v + p.rolling / p.drag) * std::exp(-2 * p.drag * s) -
                          p.rolling / p.drag - 2 * gravity * height(s);
        const double bank = -std::atan2(v2 * yawS, gravity);
        return std::array<double, 4>{pitch(s), yaw, bank, 0};
    };
    std::vector<double> boundaries{0,
                                   riseCenter - riseWidth / 2,
                                   riseCenter + riseWidth / 2,
                                   floatCenter - floatWidth / 2,
                                   floatCenter + floatWidth / 2,
                                   length - crestOffset - crestWidth / 2,
                                   length};
    std::sort(boundaries.begin(), boundaries.end());
    for (std::size_t i = 1; i < boundaries.size(); ++i)
        if (boundaries[i] - boundaries[i - 1] < 3)
            throw std::runtime_error("Finale elements overlap; route needs more space");
    controls(p, length, value, boundaries);
    JourneyParts result;
    State next = state;
    for (std::size_t i = 1; i < boundaries.size(); ++i) {
        Program section = p;
        section.geometry.clear();
        section.initial = next;
        const double a = boundaries[i - 1], b = boundaries[i], middle = (a + b) / 2;
        for (auto k : p.geometry)
            if (k.s >= a && k.s <= b) {
                k.s -= a;
                section.geometry.push_back(k);
            }
        if (middle > riseCenter - riseWidth / 2 && middle < riseCenter + riseWidth / 2) {
            section.id = "park-rise";
            section.label = "Rising banked park turn";
            section.role = Role::Airtime;
        } else if (middle > floatCenter - floatWidth / 2 && middle < floatCenter + floatWidth / 2) {
            section.id = "park-float";
            section.label = "Floating change of direction";
            section.role = Role::Airtime;
        } else if (middle > length - crestOffset - crestWidth / 2) {
            section.id = "park-crest";
            section.label = "Broad crest over the ride crossings";
            section.role = Role::Airtime;
        } else {
            section.id = "park-turn-" + std::to_string(i);
            section.label = "Low changing-bank turn";
            section.role = Role::Journey;
        }
        section.geometricDuration = replaySpline(section, .5, cancel).back().time;
        next = shootSpline(section, .5, cancel).end;
        result.active.push_back(std::move(section));
    }
    return result;
}
} // namespace coaster
