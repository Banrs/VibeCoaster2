#include "coaster/acceleration.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <tuple>

using namespace coaster;
namespace {
int checks = 0;
void check(bool value, const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
void near(double actual, double expected, double tolerance, const char* message) {
    check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
struct Fixture {
    std::array<std::vector<double>, 3> data;
    double step;
    double start = 0;
    std::size_t seat = 0;
    explicit Fixture(double duration, double dt = .001) : step(dt) {
        const auto count = static_cast<std::size_t>(std::llround(duration / step)) + 1;
        for (auto& axis : data) axis.resize(count);
    }
    AccelerationSeries series() const { return {data[0], data[1], data[2], step, start, seat}; }
    AccelerationAssessment assess() const { return assessProcessedAccelerationF2291_25(series()); }
    void constant(std::size_t axis, double value) { std::fill(data[axis].begin(), data[axis].end(), value); }
    void line(std::size_t axis, std::initializer_list<std::pair<double, double>> vertices) {
        const std::vector<std::pair<double, double>> points(vertices);
        std::size_t next = 1;
        for (std::size_t i = 0; i < data[axis].size(); ++i) {
            const double t = i * step;
            while (next + 1 < points.size() && t > points[next].first) ++next;
            const auto [ta, a] = points[next - 1]; const auto [tb, b] = points[next];
            data[axis][i] = std::lerp(a, b, std::clamp((t - ta) / (tb - ta), 0., 1.));
        }
    }
};
bool has(const AccelerationAssessment& result, const std::string& rule) {
    return std::any_of(result.diagnostics.begin(), result.diagnostics.end(), [&](const auto& d) { return d.rule == rule; });
}
std::size_t count(const AccelerationAssessment& result, const std::string& rule) {
    return static_cast<std::size_t>(std::count_if(result.diagnostics.begin(), result.diagnostics.end(), [&](const auto& d) { return d.rule == rule; }));
}
void curveBoundary(std::size_t axis, int sign, double duration, double limit) {
    Fixture f(duration);
    f.constant(axis, sign * limit);
    check(f.assess().passed, "A verified edition-25 curve boundary is accepted");
    f.constant(axis, sign * (limit + .001));
    check(has(f.assess(), "sustained-duration"), "A 0.001 g curve exceedance fails without project grace");
    f.constant(axis, sign * (limit - .001));
    check(f.assess().passed, "A value below the edition-25 curve passes");
}
Fixture historyFixture(double negativeDuration, double pulseOffset = .4) {
    const double peak = negativeDuration + pulseOffset;
    Fixture f(peak + .5);
    if (pulseOffset == .4) f.line(2, {{0, -1}, {negativeDuration - .01, -1}, {negativeDuration, 0},
        {peak, 5.5}, {peak + .3, 5.5}, {peak + .5, 1}});
    else f.line(2, {{0, -1}, {negativeDuration - .01, -1}, {negativeDuration, 0},
        {negativeDuration + .2, 1}, {peak - .1, 1}, {peak, 5.5}, {peak + .3, 5.5}, {peak + .5, 1}});
    return f;
}
} // namespace

int main() { try {
    for (const auto [duration, limit] : std::vector<std::pair<double, double>>{
        {.2, 6}, {1, 6}, {1.5, 5}, {2, 4}, {4, 4}, {4.5, 3.5}, {5, 3}, {11.8, 3}, {11.9, 2.75}, {12, 2.5}, {15, 2.5}})
        curveBoundary(0, 1, duration, limit);
    for (const auto [duration, limit] : std::vector<std::pair<double, double>>{{.2, 2}, {.35, 1.75}, {.5, 1.5}, {4, 1.5}})
        curveBoundary(0, -1, duration, limit);
    for (int sign : {1, -1}) for (const auto [duration, limit] : std::vector<std::pair<double, double>>{{.2, 3}, {1, 3}, {1.5, 2.5}, {2, 2}, {8, 2}})
        curveBoundary(1, sign, duration, limit);
    for (const auto [duration, limit] : std::vector<std::pair<double, double>>{
        {.2, 6}, {1, 6}, {1.5, 5}, {2, 4}, {4, 4}, {4.5, 3.5}, {5, 3}, {11.8, 3}, {11.9, 2.5}, {12, 2}, {20, 2}})
        curveBoundary(2, 1, duration, limit);
    for (const auto [duration, limit] : std::vector<std::pair<double, double>>{{.2, 2}, {.35, 1.75}, {.5, 1.5}, {4, 1.5}, {5.5, 1.3}, {7, 1.1}, {10, 1.1}})
        curveBoundary(2, -1, duration, limit);

    Fixture sliced(1); sliced.start = 7.2; sliced.seat = 4;
    sliced.line(0, {{0, 0}, {.2, -3}, {.7, -3}, {1, 0}});
    const auto sliceResult = sliced.assess();
    const auto slice = std::find_if(sliceResult.diagnostics.begin(), sliceResult.diagnostics.end(), [](const auto& d) {
        return d.rule == "sustained-duration" && d.axis == "X" && d.sign == "-" && std::abs(d.actual - 2) < 1e-12 && std::abs(d.limit - 1.5) < 1e-12;
    });
    check(slice != sliceResult.diagnostics.end(), "A duration slice has its own magnitude and duration assessment");
    near(slice->startTimeSeconds, 7.2 + 2. / 15, 1e-10, "Slice entry is interpolated between simulation samples");
    near(slice->endTimeSeconds, 8, 1e-10, "Slice exit is interpolated between simulation samples");
    check(slice->seatIndex == 4 && slice->clause == "7.1.4", "Diagnostics preserve the seat and governing clause");
    Fixture separate(.61);
    separate.line(0, {{0, -1.8}, {.3, -1.8}, {.305, -1.4}, {.31, -1.8}, {.61, -1.8}});
    check(separate.assess().passed, "A real 10 ms relief gap is not merged into one duration event");
    separate.line(0, {{0, -1.8}, {.3, -1.8}, {.305, -1.6}, {.31, -1.8}, {.61, -1.8}});
    check(has(separate.assess(), "sustained-duration"), "A shallower valley remains a continuous lower-level exposure");
    Fixture impact(.199); impact.constant(0, 6.01);
    auto result = impact.assess();
    check(!result.passed && has(result, "impact-review-required") && !has(result, "sustained-duration"), "Sub-200 ms single-axis over-limit exposure requires impact review");
    Fixture sustained(.2); sustained.constant(0, 6.01);
    check(has(sustained.assess(), "sustained-duration"), "Exactly 200 ms belongs to sustained acceleration");

    for (double peakSeparation : {.1, .2, .201}) {
        Fixture reversal(.8);
        reversal.line(0, {{0, 4}, {.25, 4}, {.25 + peakSeparation, -1.5}, {.8, -1.5}});
        const auto r = reversal.assess();
        check(count(r, "rapid-reversal-peak") == (peakSeparation <= .2 ? 2u : 0u), "Reversal timing uses consecutive sustained peaks with explicit 200 ms equality policy");
        if (peakSeparation == .1) for (const auto& d : r.diagnostics) if (d.rule == "rapid-reversal-peak") {
            near(d.startTimeSeconds, .25, 1e-10, "A flat predecessor peak uses its last peak time");
            near(d.endTimeSeconds, .35, 1e-10, "The next flat peak uses its first peak time");
        }
    }
    Fixture reducedPeak(.7); reducedPeak.line(0, {{0, 3}, {.25, 3}, {.35, -1}, {.7, -1}});
    check(reducedPeak.assess().passed, "The signed 50 percent peak limits are inclusive");
    Fixture briefReverse(.6); briefReverse.line(0, {{0, 4}, {.2, 4}, {.25, -1}, {.3, 4}, {.6, 4}});
    check(!has(briefReverse.assess(), "rapid-reversal-peak"), "An intervening impact is not a sustained opposite-sign reversal");

    for (int sign : {1, -1}) {
        Fixture ellipse(.3); ellipse.constant(0, sign > 0 ? 4.8 : -1.6); ellipse.constant(1, 1.8);
        check(ellipse.assess().passed, "The signed quadrant ellipse boundary is inclusive");
        ellipse.constant(1, 1.801);
        check(has(ellipse.assess(), "combined-ellipse"), "Simultaneous axes can fail despite acceptable individual values");
    }
    for (double duration : {.199, .2, .201}) {
        Fixture ellipse(duration); ellipse.constant(0, 4.8); ellipse.constant(1, 1.81);
        check(has(ellipse.assess(), "combined-ellipse") == (duration >= .2), "Combined excursions below 200 ms are explicitly excluded");
        check(!has(ellipse.assess(), "impact-review-required"), "Combined exclusion does not invent a single-axis impact failure");
    }
    Fixture ellipseCrossing(1); ellipseCrossing.line(0, {{0, 0}, {1, 6}}); ellipseCrossing.constant(1, 1.8);
    result = ellipseCrossing.assess();
    const auto pair = std::find_if(result.diagnostics.begin(), result.diagnostics.end(), [](const auto& d) { return d.rule == "combined-ellipse"; });
    check(pair != result.diagnostics.end(), "A combined excursion of exactly 200 ms fails");
    near(pair->startTimeSeconds, .8, 1e-10, "Quadratic ellipse intersection gives the analytic crossing time");
    near(pair->endTimeSeconds, 1, 1e-10, "The complete terminal combined exposure is retained");
    near(pair->utilization, std::sqrt(1.36), 1e-10, "Ellipse utilization is normalized radial magnitude");

    auto exactHistory = historyFixture(3).assess();
    check(!has(exactHistory, "negative-history-duration"), "Exactly three seconds negative does not activate history reduction");
    auto longHistory = historyFixture(3.001).assess();
    check(has(longHistory, "negative-history-duration"), "Strictly more than three seconds activates the verified 5 g reduced intercept");
    check(!has(longHistory, "weightless-to-positive-transition"), "The history fixture independently satisfies the 133 ms transition rule");
    for (const auto [duration, limit] : std::vector<std::pair<double, double>>{{.2, 5}, {1.5, 5}, {1.75, 4.5}, {2, 4}, {2.25, 3}, {2.5, 2}, {3, 2}}) {
        for (double delta : {0., .001}) {
            Fixture boundary(3.101 + duration);
            boundary.line(2, {{0, -1}, {3.09, -1}, {3.1, 0}, {3.101, limit + delta}, {3.101 + duration, limit + delta}});
            check(has(boundary.assess(), "negative-history-duration") == (delta > 0), "Every reduced positive-Z vertex/interpolation is independently bounded without borrowing normal limits");
        }
    }
    check(has(historyFixture(3.1, 5.5).assess(), "negative-history-duration"), "The positive reduction remains active before six seconds elapse");
    check(!has(historyFixture(3.1, 6.2).assess(), "negative-history-duration"), "Normal positive limits resume after the six-second history window");
    Fixture historyPair(4.1); historyPair.constant(0, 3);
    historyPair.line(2, {{0, -1}, {3.09, -1}, {3.1, 0}, {3.5, 4.5}, {3.8, 4.5}, {4.1, 1}});
    check(has(historyPair.assess(), "combined-ellipse-negative-history"), "The documented conservative history policy reduces the positive Z ellipse radius to five");
    for (double transition : {.132, .133, .134}) {
        Fixture z(.25 + transition); z.line(2, {{0, 0}, {.25, 0}, {.25 + transition, 2}});
        check(has(z.assess(), "weightless-to-positive-transition") == (transition < .133), "The zero-to-two crossing requires at least 133 ms, including an exact two-g endpoint");
    }
    Fixture notSustained(.299); notSustained.line(2, {{0, 0}, {.199, 0}, {.299, 2}});
    check(!has(notSustained.assess(), "weightless-to-positive-transition"), "A 199 ms weightless episode does not trigger the sustained transition rule");
    Fixture rest(91); rest.constant(2, 1);
    check(rest.assess().passed, "Stationary one-g baseline does not fabricate a long-exposure failure");
    Fixture longZ(40.001); longZ.constant(2, 1.01);
    check(has(longZ.assess(), "positive-tail-review-required"), "The unresolved positive-Z tail is reported beyond forty seconds above baseline");
    Fixture longX(90.001); longX.constant(0, 1);
    check(has(longX.assess(), "duration-over-90s-review-required"), "The scope does not silently extrapolate dynamic exposures beyond ninety seconds");

    for (double step : {1. / 960, 1. / 1920, 1. / 997}) {
        Fixture ramp(1, step);
        for (std::size_t i = 0; i < ramp.data[0].size(); ++i) ramp.data[0][i] = 3 * i * step - 1;
        result = ramp.assess();
        check(result.onset100ms[0].available, "A full centered onset window is available");
        near(result.onset100ms[0].minimumGps, 3, 2e-8, "Centered least squares recovers an independent linear-ramp slope");
        near(result.onset100ms[0].maximumGps, 3, 2e-8, "Rolling onset retains a constant analytic slope across the record");
        check(result.onset100ms[0].minimumTimeSeconds >= .05 - 1e-12 && result.onset100ms[0].maximumTimeSeconds <= 1 - .05 + 1e-12,
            "Onset extrema do not use incomplete end windows");
    }
    Fixture equilibrium(2, 1. / 960); equilibrium.constant(0, .7); equilibrium.constant(1, -.2); equilibrium.constant(2, 1);
    result = assessAccelerationF2291_25(equilibrium.series());
    check(result.passed, "A constant supported seat-force record passes filtering and assessment");
    for (std::size_t axis = 0; axis < 3; ++axis) {
        near(result.filteredExtrema[axis].minimumG, equilibrium.data[axis][0], 2e-11, "Filter equilibrium does not create a startup dip");
        near(result.filteredExtrema[axis].maximumG, equilibrium.data[axis][0], 2e-11, "Filter equilibrium does not create a startup overshoot");
        near(result.onset100ms[axis].minimumGps, 0, 2e-8, "Equilibrium has no onset transient");
        near(result.onset100ms[axis].maximumGps, 0, 2e-8, "Equilibrium preserves zero onset");
    }
    Fixture filteredRest(91, 1. / 960); filteredRest.constant(2, 1);
    result = assessAccelerationF2291_25(filteredRest.series());
    check(result.passed, "Long filtered equilibrium does not turn numerical roundoff into dynamic tail exposure");
    near(result.onset100ms[2].maximumGps, 0, 2e-8, "Long equilibrium preserves rolling onset accuracy");
    Fixture longRamp(180, 1. / 960);
    for (std::size_t i = 0; i < longRamp.data[0].size(); ++i) longRamp.data[0][i] = -.5 + .01 * i * longRamp.step;
    result = longRamp.assess();
    near(result.onset100ms[0].minimumGps, .01, 2e-7, "Long-record rolling least squares retains its independent slope oracle");
    near(result.onset100ms[0].maximumGps, .01, 2e-7, "Rolling onset does not accumulate a material long-record slope drift");
    Fixture sine(12, 1. / 960);
    for (std::size_t i = 0; i < sine.data[0].size(); ++i) {
        const double time = i * sine.step;
        sine.data[0][i] = .3 * (1 - std::exp(-time)) * std::sin(2 * std::numbers::pi * 5 * time);
    }
    result = assessAccelerationF2291_25(sine.series());
    near(result.filteredExtrema[0].maximumG, .3 / std::sqrt(2.), .0002, "Single-pass four-pole processing has minus-three-dB amplitude at the prewarped five-Hz corner");
    near(result.filteredExtrema[0].minimumG, -.3 / std::sqrt(2.), .0002, "The cutoff response is symmetric without doubled forward-backward attenuation");
    Fixture stepInput(2, 1. / 960);
    for (std::size_t i = 960; i < stepInput.data[0].size(); ++i) stepInput.data[0][i] = .3;
    result = assessAccelerationF2291_25(stepInput.series());
    check(result.onset100ms[0].maximumTimeSeconds > 1 && result.filteredExtrema[0].maximumG > .3, "Causal Butterworth response retains its delay and physical filter overshoot");

    AccelerationSeries invalid;
    check(!assessAccelerationF2291_25(invalid).performed, "Empty data is rejected, not reported as a pass");
    auto mismatch = equilibrium.series(); mismatch.lateral = mismatch.lateral.first(1);
    check(!assessAccelerationF2291_25(mismatch).performed, "Mismatched channel lengths are rejected");
    auto badStep = equilibrium.series(); badStep.stepSeconds = .1;
    check(!assessAccelerationF2291_25(badStep).performed, "Nyquist must exceed the five-Hz corner");
    Fixture nonfinite(.5); nonfinite.data[1][30] = INFINITY;
    check(!nonfinite.assess().performed, "Nonfinite input is rejected before filtering");
    result = assessAccelerationF2291_25(equilibrium.series(), [] { return true; });
    check(result.cancelled && !result.performed && !result.passed, "Cancellation never becomes a successful assessment");
    int calls = 0;
    result = assessAccelerationF2291_25(equilibrium.series(), [&] { return ++calls > 5; });
    check(result.cancelled && !result.performed && calls > 5, "Cancellation propagates while filtering");
    calls = 0;
    result = assessProcessedAccelerationF2291_25(sine.series(), [&] { return ++calls > 30; });
    check(result.cancelled && !result.passed && calls > 30, "Cancellation propagates through processed-data assessment");
    const auto again = sliced.assess();
    check(again.diagnostics.size() == sliceResult.diagnostics.size(), "Diagnostic count is deterministic");
    for (std::size_t i = 0; i < again.diagnostics.size(); ++i) {
        const auto& a = again.diagnostics[i]; const auto& b = sliceResult.diagnostics[i];
        check(std::tie(a.clause, a.rule, a.axis, a.sign, a.seatIndex, a.startTimeSeconds, a.endTimeSeconds, a.actual, a.limit, a.utilization) ==
              std::tie(b.clause, b.rule, b.axis, b.sign, b.seatIndex, b.startTimeSeconds, b.endTimeSeconds, b.actual, b.limit, b.utilization),
              "Repeated assessment produces identically ordered clause-level evidence");
    }
    std::cout << "PASS " << checks << " scoped F2291-25 acceleration checks\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << "FAIL after " << checks << ": " << error.what() << '\n';
    return 1;
} }
