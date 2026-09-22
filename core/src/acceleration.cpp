#include "coaster/acceleration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <tuple>

namespace coaster {
namespace {

constexpr double sustainedSeconds = .2;
constexpr double timeRoundoff = 1e-10;
constexpr double valueRoundoff = 1e-12;
constexpr std::array<const char *, 3> axisNames{"X", "Y", "Z"};

bool atLeast(double actual, double minimum) {
    return actual + timeRoundoff >= minimum;
}

bool exceeds(double actual, double limit) {
    return actual > limit + valueRoundoff * std::max(1., std::abs(limit));
}

struct Run {
    double begin{};
    double end{};
    double peak{};
    double firstPeak{};
    double lastPeak{};
};

struct LimitPoint {
    double seconds;
    double magnitudeG;
};
struct Curve {
    std::span<const LimitPoint> points;
    double limit(double seconds) const {
        if (seconds <= points.front().seconds)
            return points.front().magnitudeG;
        for (std::size_t i = 1; i < points.size(); ++i)
            if (seconds <= points[i].seconds) {
                const auto a = points[i - 1], b = points[i];
                return std::lerp(a.magnitudeG, b.magnitudeG, (seconds - a.seconds) / (b.seconds - a.seconds));
            }
        return points.back().magnitudeG;
    }
};

// Edition 25 base-case figure coordinates, visually transcribed from the
// original figure reproduction, not from a different edition's tables.
constexpr std::array xPositive{LimitPoint{.2, 6}, LimitPoint{1, 6},    LimitPoint{2, 4},   LimitPoint{4, 4},
                               LimitPoint{5, 3},  LimitPoint{11.8, 3}, LimitPoint{12, 2.5}};
constexpr std::array xNegative{LimitPoint{.2, 2}, LimitPoint{.5, 1.5}};
constexpr std::array yEither{LimitPoint{.2, 3}, LimitPoint{1, 3}, LimitPoint{2, 2}};
constexpr std::array zPositive{LimitPoint{.2, 6}, LimitPoint{1, 6},    LimitPoint{2, 4}, LimitPoint{4, 4},
                               LimitPoint{5, 3},  LimitPoint{11.8, 3}, LimitPoint{12, 2}};
constexpr std::array zReduced{LimitPoint{.2, 5}, LimitPoint{1.5, 5}, LimitPoint{2, 4}, LimitPoint{2.5, 2}};
constexpr std::array zNegative{LimitPoint{.2, 2}, LimitPoint{.5, 1.5}, LimitPoint{4, 1.5},
                               LimitPoint{7, 1.1}};

// The connected superlevel sets of the piecewise-linear record. A genuine
// below-threshold interval always separates events, regardless of its length.
template <class Callback>
bool scanRuns(std::span<const double> values, double step, int sign, double level, bool strict,
              const std::function<bool()> &cancel, Callback callback, double from = 0,
              double to = std::numeric_limits<double>::infinity()) {
    auto inside = [&](double x) { return strict ? x > level : x >= level; };
    Run run{};
    bool active = false;
    if (cancel && cancel())
        return false;
    if (from >= static_cast<double>(values.size() - 1) * step || to <= from)
        return true;
    const auto first = static_cast<std::size_t>(std::max(0., std::floor(from / step)));
    for (std::size_t i = first; i + 1 < values.size(); ++i) {
        if ((i & 4095) == 0 && cancel && cancel())
            return false;
        const double sampleTime = static_cast<double>(i) * step;
        if (sampleTime >= to)
            break;
        const double t = std::max(from, sampleTime), last = std::min(to, sampleTime + step);
        if (last <= t)
            continue;
        const double a = sign * std::lerp(values[i], values[i + 1], (t - sampleTime) / step);
        const double b = sign * std::lerp(values[i], values[i + 1], (last - sampleTime) / step);
        const bool inA = inside(a), inB = inside(b);
        if (!inA && !inB)
            continue;
        const double crossing = a == b ? t : t + (last - t) * (level - a) / (b - a);
        const double begin = inA ? t : crossing;
        const double end = inB ? last : crossing;
        if (!active) {
            run = {begin, end, level, begin, begin};
            active = true;
        }
        auto peak = [&](double v, double time) {
            if (v > run.peak) {
                run.peak = v;
                run.firstPeak = time;
                run.lastPeak = time;
            } else if (v == run.peak)
                run.lastPeak = time;
        };
        if (inA)
            peak(a, t);
        if (inB)
            peak(b, last);
        run.end = end;
        if (!inB) {
            if (run.end > run.begin)
                callback(run);
            active = false;
        }
    }
    if (active && run.end > run.begin)
        callback(run);
    return true;
}

struct Biquad {
    double b0{}, b1{}, b2{}, a1{}, a2{}, z1{}, z2{};
    Biquad(double step, double q, double equilibrium) {
        const double k = std::tan(std::numbers::pi * 5 * step);
        const double normalization = 1 / (1 + k / q + k * k);
        b0 = k * k * normalization;
        b1 = 2 * b0;
        b2 = b0;
        a1 = 2 * (k * k - 1) * normalization;
        a2 = (1 - k / q + k * k) * normalization;
        z1 = (1 - b0) * equilibrium;
        z2 = (b2 - a2) * equilibrium;
    }
    double process(double input) {
        const double output = b0 * input + z1;
        z1 = b1 * input - a1 * output + z2;
        z2 = b2 * input - a2 * output;
        return output;
    }
};

bool filter(std::span<const double> input, double step, std::vector<double> &output,
            const std::function<bool()> &cancel) {
    Biquad first(step, 1 / (2 * std::cos(std::numbers::pi / 8)), input.front());
    Biquad second(step, 1 / (2 * std::cos(3 * std::numbers::pi / 8)), input.front());
    output.resize(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) {
        if ((i & 4095) == 0 && cancel && cancel())
            return false;
        output[i] = second.process(first.process(input[i]));
    }
    return true;
}

bool measure(std::span<const double> values, double step, double start, AccelerationExtrema &extrema,
             AccelerationOnset &onset, const std::function<bool()> &cancel) {
    extrema = {values.front(), values.front(), start, start};
    for (std::size_t i = 0; i < values.size(); ++i) {
        if ((i & 4095) == 0 && cancel && cancel())
            return false;
        const double time = start + static_cast<double>(i) * step;
        if (values[i] < extrema.minimumG) {
            extrema.minimumG = values[i];
            extrema.minimumTimeSeconds = time;
        }
        if (values[i] > extrema.maximumG) {
            extrema.maximumG = values[i];
            extrema.maximumTimeSeconds = time;
        }
    }
    // Sliding discrete least-squares fit. Include interpolated values exactly
    // at +/-50 ms when those boundaries fall between actual uniform samples.
    // Only complete centered windows are reported; no end padding is invented.
    constexpr double half = .05;
    const double halfSteps = half / step;
    if (halfSteps > static_cast<double>(values.size()))
        return true;
    const auto k = static_cast<std::size_t>(std::floor(halfSteps + 1e-12));
    const bool interpolate = std::abs(static_cast<double>(k) * step - half) > 1e-12;
    const std::size_t edge = k + (interpolate ? 1 : 0);
    if (edge >= values.size() || edge > (values.size() - 1) / 2)
        return true;
    const std::size_t count = 2 * k + 1;
    double sum = 0, moment = 0;
    for (std::size_t j = 0; j < count; ++j) {
        sum += values[edge - k + j];
        moment += (static_cast<double>(j) - static_cast<double>(k)) * values[edge - k + j];
    }
    const double n = static_cast<double>(count);
    const double denominator = step * step * n * (n * n - 1) / 12 + (interpolate ? 2 * half * half : 0);
    if (!(denominator > 0))
        return true;
    for (std::size_t center = edge; center + edge < values.size(); ++center) {
        if ((center & 4095) == 0 && cancel && cancel())
            return false;
        double numerator = step * moment;
        if (interpolate) {
            const double fraction = halfSteps - static_cast<double>(k);
            const double left = std::lerp(values[center - k], values[center - k - 1], fraction);
            const double right = std::lerp(values[center + k], values[center + k + 1], fraction);
            numerator += half * (right - left);
        }
        const double slope = numerator / denominator;
        const double time = start + static_cast<double>(center) * step;
        if (!onset.available || slope < onset.minimumGps) {
            onset.minimumGps = slope;
            onset.minimumTimeSeconds = time;
        }
        if (!onset.available || slope > onset.maximumGps) {
            onset.maximumGps = slope;
            onset.maximumTimeSeconds = time;
        }
        onset.available = true;
        if (center + edge + 1 < values.size()) {
            const double removed = values[center - k], added = values[center + k + 1];
            moment += -sum + (static_cast<double>(k) + 1) * removed + static_cast<double>(k) * added;
            sum += added - removed;
        }
    }
    return true;
}

std::array<std::span<const double>, 3> axes(const AccelerationSeries &series) {
    return {series.longitudinal, series.lateral, series.vertical};
}

void diagnose(AccelerationAssessment &result, const AccelerationSeries &series, const char *clause,
              const char *rule, std::string axis, std::string sign, double begin, double end, double actual,
              double limit, bool minimum = false) {
    const double utilization = minimum ? (actual > 0 ? limit / actual : std::numeric_limits<double>::max())
                                       : (limit > 0 ? actual / limit : 0);
    result.diagnostics.push_back({clause, rule, std::move(axis), std::move(sign), series.seatIndex,
                                  series.startTimeSeconds + begin, series.startTimeSeconds + end, actual,
                                  limit, utilization});
}

bool validate(const AccelerationSeries &series, AccelerationAssessment &result,
              const std::function<bool()> &cancel) {
    if (cancel && cancel()) {
        result.cancelled = true;
        return false;
    }
    const std::size_t size = series.longitudinal.size();
    if (size < 2 || series.lateral.size() != size || series.vertical.size() != size ||
        !std::isfinite(series.stepSeconds) || series.stepSeconds <= 0 || series.stepSeconds >= .1 ||
        !std::isfinite(series.startTimeSeconds) ||
        !std::isfinite(series.startTimeSeconds + static_cast<double>(size - 1) * series.stepSeconds)) {
        diagnose(result, series, "7.1.1.2", "invalid-uniform-series", "input", "", 0, 0, 0, 0);
        return false;
    }
    for (const auto values : axes(series))
        for (std::size_t i = 0; i < size; ++i) {
            if ((i & 4095) == 0 && cancel && cancel()) {
                result.cancelled = true;
                return false;
            }
            if (!std::isfinite(values[i])) {
                diagnose(result, series, "7.1.1.2", "nonfinite-acceleration", "input", "",
                         i * series.stepSeconds, i * series.stepSeconds, 0, 0);
                return false;
            }
        }
    return true;
}

using Curves = std::array<std::array<Curve, 2>, 3>; // positive, negative
const Curves baseCurves{std::array{Curve{xPositive}, Curve{xNegative}},
                        std::array{Curve{yEither}, Curve{yEither}},
                        std::array{Curve{zPositive}, Curve{zNegative}}};
struct TimeWindow {
    double begin;
    double end;
};

bool assessAxis(const AccelerationSeries &series, AccelerationAssessment &result, std::size_t axis, int sign,
                Curve curve, const char *clause, const char *rule, const std::function<bool()> &cancel,
                double from = 0, double to = std::numeric_limits<double>::infinity()) {
    const auto values = axes(series)[axis];
    const auto &extrema = result.filteredExtrema[axis];
    const double maximum = sign > 0 ? extrema.maximumG : -extrema.minimumG;
    if (maximum <= 0)
        return true;
    const double cap = curve.points.front().magnitudeG;
    const double floor =
        std::min_element(curve.points.begin(), curve.points.end(), [](const auto &a, const auto &b) {
            return a.magnitudeG < b.magnitudeG;
        })->magnitudeG;
    // A slice at or below every point of the duration curve cannot fail that
    // curve. Reversal/history and long-duration scope checks remain separate.
    if (maximum <= floor)
        return true;
    std::vector<double> levels;
    // At most 0.1 g between slice lines. Beyond the 200 ms intercept, the
    // cap-crossing scan below already establishes failure or impact review,
    // so potentially huge invalid input does not create unbounded slice work.
    const double top = std::min(cap, maximum);
    for (int i = 1; i <= static_cast<int>(std::floor(top * 10)); ++i)
        levels.push_back(i * .1);
    levels.push_back(top);
    for (const auto point : curve.points)
        if (point.magnitudeG <= top)
            levels.push_back(point.magnitudeG);
    std::sort(levels.begin(), levels.end());
    levels.erase(std::unique(levels.begin(), levels.end()), levels.end());
    const char *signName = sign > 0 ? "+" : "-";
    for (const double level : levels) {
        if (level <= floor)
            continue;
        if (!scanRuns(
                values, series.stepSeconds, sign, level, false, cancel,
                [&](const Run &run) {
                    const double duration = run.end - run.begin;
                    if (!atLeast(duration, sustainedSeconds))
                        return;
                    const double limit = curve.limit(duration);
                    if (exceeds(level, limit))
                        diagnose(result, series, clause, rule, axisNames[axis], signName, run.begin, run.end,
                                 level, limit);
                },
                from, to))
            return false;
    }
    if (exceeds(maximum, cap) &&
        !scanRuns(
            values, series.stepSeconds, sign, cap, true, cancel,
            [&](const Run &run) {
                if (!exceeds(run.peak, cap))
                    return;
                const bool sustained = atLeast(run.end - run.begin, sustainedSeconds);
                diagnose(result, series, sustained ? clause : "7.1.4.2",
                         sustained ? rule : "impact-review-required", axisNames[axis], signName, run.begin,
                         run.end, run.peak, cap);
            },
            from, to))
        return false;
    return true;
}

bool assessHistory(const AccelerationSeries &series, AccelerationAssessment &result,
                   std::vector<TimeWindow> &windows, const std::function<bool()> &cancel) {
    const auto values = series.vertical;
    const double end = static_cast<double>(values.size() - 1) * series.stepSeconds;
    std::vector<Run> negative, weightless;
    if (!scanRuns(values, series.stepSeconds, -1, 0, true, cancel, [&](const Run &run) {
            if (run.end - run.begin > 3 + timeRoundoff)
                negative.push_back(run);
        }))
        return false;
    if (!scanRuns(values, series.stepSeconds, -1, 0, false, cancel, [&](const Run &run) {
            if (atLeast(run.end - run.begin, sustainedSeconds))
                weightless.push_back(run);
        }))
        return false;
    std::size_t negativeIndex = 0, weightlessIndex = 0;
    bool qualifiedTransition = false;
    bool completed = true;
    if (!scanRuns(values, series.stepSeconds, 1, 0, true, cancel, [&](const Run &run) {
            bool historyTrigger = false;
            while (negativeIndex < negative.size() &&
                   negative[negativeIndex].end <= run.begin + timeRoundoff) {
                historyTrigger = true;
                ++negativeIndex;
            }
            if (historyTrigger) {
                const double until = std::min(end, run.begin + 6);
                if (!windows.empty() && run.begin <= windows.back().end)
                    windows.back().end = until;
                else
                    windows.push_back({run.begin, until});
            }
            while (weightlessIndex < weightless.size() &&
                   weightless[weightlessIndex].end <= run.begin + timeRoundoff) {
                qualifiedTransition = true;
                ++weightlessIndex;
            }
            if (!qualifiedTransition || run.peak < 2 || !completed)
                return;
            const auto first =
                static_cast<std::size_t>(std::max(0., std::floor(run.begin / series.stepSeconds)));
            for (std::size_t i = first; i + 1 < values.size() && i * series.stepSeconds < run.end; ++i) {
                if ((i & 4095) == 0 && cancel && cancel()) {
                    completed = false;
                    return;
                }
                if (values[i] < 2 && values[i + 1] >= 2) {
                    const double crossing =
                        (static_cast<double>(i) + (2 - values[i]) / (values[i + 1] - values[i])) *
                        series.stepSeconds;
                    if (!atLeast(crossing - run.begin, .133))
                        diagnose(result, series, "7.1.7.2", "weightless-to-positive-transition", "Z", "+",
                                 run.begin, crossing, crossing - run.begin, .133, true);
                    qualifiedTransition = false;
                    break;
                }
            }
        }))
        return false;
    return completed;
}

bool assessReversals(const AccelerationSeries &series, AccelerationAssessment &result, std::size_t axis,
                     const std::function<bool()> &cancel) {
    std::array<std::vector<Run>, 2> runs;
    const auto values = axes(series)[axis];
    for (int side = 0; side < 2; ++side)
        if (!scanRuns(values, series.stepSeconds, side == 0 ? 1 : -1, 0, true, cancel, [&](const Run &run) {
                if (atLeast(run.end - run.begin, sustainedSeconds))
                    runs[side].push_back(run);
            }))
            return false;
    std::array<std::size_t, 2> next{};
    Run previous{};
    int previousSide = -1;
    while (next[0] < runs[0].size() || next[1] < runs[1].size()) {
        if (cancel && cancel())
            return false;
        const int side = next[1] == runs[1].size()                         ? 0
                         : next[0] == runs[0].size()                       ? 1
                         : runs[0][next[0]].begin < runs[1][next[1]].begin ? 0
                                                                           : 1;
        const Run current = runs[side][next[side]++];
        if (previousSide >= 0 && side != previousSide &&
            current.firstPeak - previous.lastPeak <= sustainedSeconds + timeRoundoff) {
            // Plateau peaks use their nearest endpoints. Exactly 200 ms takes
            // the reduced limit because the clause's first sentence says >200.
            for (const auto entry : {std::pair{previous, previousSide}, std::pair{current, side}}) {
                const double limit = .5 * baseCurves[axis][entry.second].limit(sustainedSeconds);
                if (exceeds(entry.first.peak, limit))
                    diagnose(result, series, "7.1.6.1", "rapid-reversal-peak", axisNames[axis],
                             entry.second == 0 ? "+" : "-", previous.lastPeak, current.firstPeak,
                             entry.first.peak, limit);
            }
        }
        previous = current;
        previousSide = side;
    }
    return true;
}

bool assessLongDurationScope(const AccelerationSeries &series, AccelerationAssessment &result,
                             const std::function<bool()> &cancel) {
    for (std::size_t axis = 0; axis < 3; ++axis)
        for (int sign : {1, -1}) {
            const bool positiveZ = axis == 2 && sign > 0;
            // Equilibrium filter roundoff must not turn resting 1 g (or horizontal
            // zero) into an uninterrupted dynamic exposure lasting the whole ride.
            const double threshold = (positiveZ ? 1. : 0.) + valueRoundoff;
            const double maximum = positiveZ ? 40 : 90;
            if (!scanRuns(axes(series)[axis], series.stepSeconds, sign, threshold, true, cancel,
                          [&](const Run &run) {
                              if (run.end - run.begin > maximum + timeRoundoff)
                                  diagnose(result, series, "7.1.4.6",
                                           positiveZ ? "positive-tail-review-required"
                                                     : "duration-over-90s-review-required",
                                           axisNames[axis], sign > 0 ? "+" : "-", run.begin, run.end,
                                           run.end - run.begin, maximum);
                          }))
                return false;
        }
    return true;
}

bool assessPair(const AccelerationSeries &series, AccelerationAssessment &result, std::size_t firstAxis,
                std::size_t secondAxis, const std::vector<TimeWindow> &windows,
                const std::function<bool()> &cancel) {
    const auto x = axes(series)[firstAxis], y = axes(series)[secondAxis];
    const std::string axis = std::string(axisNames[firstAxis]) + axisNames[secondAxis];
    auto normalizedBound = [&](std::size_t a) {
        const auto &e = result.filteredExtrema[a];
        const double positive = a == 2 && !windows.empty() ? zReduced.front().magnitudeG
                                                           : baseCurves[a][0].points.front().magnitudeG;
        return std::max(std::max(0., e.maximumG) / positive,
                        std::max(0., -e.minimumG) / baseCurves[a][1].points.front().magnitudeG);
    };
    const double boundX = normalizedBound(firstAxis), boundY = normalizedBound(secondAxis);
    // A rectangle enclosing the complete paired trace inside the smallest
    // applicable ellipse is a sufficient proof for every point in the pair.
    if (boundX * boundX + boundY * boundY <= 1)
        return true;
    Run run{};
    bool active = false;
    std::string peakSign;
    bool peakReduced = false;
    auto close = [&] {
        if (active && atLeast(run.end - run.begin, sustainedSeconds))
            diagnose(result, series, "7.1.5.1",
                     peakReduced ? "combined-ellipse-negative-history" : "combined-ellipse", axis, peakSign,
                     run.begin, run.end, run.peak, 1);
        active = false;
    };
    std::size_t windowIndex = 0;
    for (std::size_t i = 0; i + 1 < x.size(); ++i) {
        if ((i & 4095) == 0 && cancel && cancel())
            return false;
        const double time = static_cast<double>(i) * series.stepSeconds;
        while (windowIndex < windows.size() && windows[windowIndex].end <= time)
            ++windowIndex;
        std::array<double, 8> cuts{};
        std::size_t cutCount = 0;
        cuts[cutCount++] = 0;
        cuts[cutCount++] = 1;
        for (const auto values : {x, y})
            if ((values[i] < 0 && values[i + 1] > 0) || (values[i] > 0 && values[i + 1] < 0))
                cuts[cutCount++] = -values[i] / (values[i + 1] - values[i]);
        if (secondAxis == 2)
            for (std::size_t w = windowIndex;
                 w < windows.size() && windows[w].begin < time + series.stepSeconds; ++w) {
                for (double boundary : {windows[w].begin, windows[w].end})
                    if (boundary > time && boundary < time + series.stepSeconds)
                        cuts[cutCount++] = (boundary - time) / series.stepSeconds;
            }
        std::sort(cuts.begin(), cuts.begin() + cutCount);
        cutCount =
            static_cast<std::size_t>(std::unique(cuts.begin(), cuts.begin() + cutCount) - cuts.begin());
        for (std::size_t part = 1; part < cutCount; ++part) {
            const double a = cuts[part - 1], b = cuts[part], middle = (a + b) * .5;
            const double midX = std::lerp(x[i], x[i + 1], middle), midY = std::lerp(y[i], y[i + 1], middle);
            const double midTime = time + middle * series.stepSeconds;
            bool reduced = false;
            if (secondAxis == 2 && midY > 0)
                for (std::size_t w = windowIndex; w < windows.size() && windows[w].begin <= midTime; ++w)
                    reduced = reduced || midTime < windows[w].end;
            const double radiusX = baseCurves[firstAxis][midX >= 0 ? 0 : 1].limit(sustainedSeconds);
            const double radiusY = reduced
                                       ? zReduced.front().magnitudeG
                                       : baseCurves[secondAxis][midY >= 0 ? 0 : 1].limit(sustainedSeconds);
            const double x0 = std::lerp(x[i], x[i + 1], a) / radiusX,
                         x1 = std::lerp(x[i], x[i + 1], b) / radiusX;
            const double y0 = std::lerp(y[i], y[i + 1], a) / radiusY,
                         y1 = std::lerp(y[i], y[i + 1], b) / radiusY;
            // Squared ellipse utilization is convex along this linear segment.
            if (std::max(x0 * x0 + y0 * y0, x1 * x1 + y1 * y1) <= 1) {
                close();
                continue;
            }
            const double dx = x1 - x0, dy = y1 - y0;
            const double qa = dx * dx + dy * dy, qb = 2 * (x0 * dx + y0 * dy), qc = x0 * x0 + y0 * y0 - 1;
            std::array<double, 4> boundaries{0, 1};
            std::size_t count = 2;
            auto root = [&](double r) {
                if (r > 0 && r < 1)
                    boundaries[count++] = r;
            };
            if (qa > 0) {
                const double discriminant = qb * qb - 4 * qa * qc;
                if (discriminant >= 0) {
                    const double q = -.5 * (qb + std::copysign(std::sqrt(discriminant), qb));
                    if (q != 0) {
                        root(q / qa);
                        root(qc / q);
                    } else
                        root(-qb / (2 * qa));
                }
            } else if (qb != 0)
                root(-qc / qb);
            std::sort(boundaries.begin(), boundaries.begin() + count);
            count = static_cast<std::size_t>(std::unique(boundaries.begin(), boundaries.begin() + count) -
                                             boundaries.begin());
            for (std::size_t sub = 1; sub < count; ++sub) {
                const double u = boundaries[sub - 1], v = boundaries[sub], mid = (u + v) * .5;
                const double begin = time + std::lerp(a, b, u) * series.stepSeconds;
                const double end = time + std::lerp(a, b, v) * series.stepSeconds;
                if ((qa * mid + qb) * mid + qc <= valueRoundoff) {
                    close();
                    continue;
                }
                if (!active) {
                    run = {begin, end, 0, begin, begin};
                    active = true;
                }
                run.end = end;
                for (double position : {u, v}) {
                    const double px = std::lerp(x0, x1, position), py = std::lerp(y0, y1, position);
                    const double utilization = std::hypot(px, py);
                    if (utilization > run.peak) {
                        run.peak = utilization;
                        peakSign = std::string(px >= 0 ? "+" : "-") + (py >= 0 ? "+" : "-");
                        peakReduced = reduced;
                    }
                }
            }
        }
    }
    close();
    return true;
}

void finish(AccelerationAssessment &result) {
    std::sort(result.diagnostics.begin(), result.diagnostics.end(), [](const auto &a, const auto &b) {
        return std::tie(a.clause, a.rule, a.axis, a.sign, a.seatIndex, a.startTimeSeconds, a.endTimeSeconds,
                        a.actual, a.limit) < std::tie(b.clause, b.rule, b.axis, b.sign, b.seatIndex,
                                                      b.startTimeSeconds, b.endTimeSeconds, b.actual,
                                                      b.limit);
    });
    result.performed = !result.cancelled;
    result.passed = result.performed && result.diagnostics.empty();
}

} // namespace

AccelerationAssessment assessProcessedAccelerationF2291_25(const AccelerationSeries &series,
                                                           const std::function<bool()> &cancel) {
    AccelerationAssessment result;
    if (!validate(series, result, cancel))
        return result;
    auto cancelled = [&] {
        result.cancelled = true;
        finish(result);
        return result;
    };
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (!measure(axes(series)[axis], series.stepSeconds, series.startTimeSeconds,
                     result.filteredExtrema[axis], result.onset100ms[axis], cancel))
            return cancelled();
    std::vector<TimeWindow> history;
    if (!assessHistory(series, result, history, cancel))
        return cancelled();
    for (std::size_t axis = 0; axis < 3; ++axis)
        for (int side = 0; side < 2; ++side)
            if (!assessAxis(series, result, axis, side == 0 ? 1 : -1, baseCurves[axis][side], "7.1.4",
                            "sustained-duration", cancel))
                return cancelled();
    for (const auto window : history)
        if (!assessAxis(series, result, 2, 1, {zReduced}, "7.1.7.1", "negative-history-duration", cancel,
                        window.begin, window.end))
            return cancelled();
    for (std::size_t axis = 0; axis < 2; ++axis)
        if (!assessReversals(series, result, axis, cancel))
            return cancelled();
    for (std::size_t a = 0; a < 2; ++a)
        for (std::size_t b = a + 1; b < 3; ++b)
            if (!assessPair(series, result, a, b, history, cancel))
                return cancelled();
    if (!assessLongDurationScope(series, result, cancel))
        return cancelled();
    finish(result);
    return result;
}

AccelerationAssessment assessAccelerationF2291_25(const AccelerationSeries &series,
                                                  const std::function<bool()> &cancel) {
    AccelerationAssessment result;
    if (!validate(series, result, cancel))
        return result;
    std::array<std::vector<double>, 3> processed;
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (!filter(axes(series)[axis], series.stepSeconds, processed[axis], cancel)) {
            result.cancelled = true;
            return result;
        }
    return assessProcessedAccelerationF2291_25({processed[0], processed[1], processed[2], series.stepSeconds,
                                                series.startTimeSeconds, series.seatIndex},
                                               cancel);
}

} // namespace coaster
