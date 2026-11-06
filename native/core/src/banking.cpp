#include "banking.hpp"
#include "authoring.hpp"
#include "curve_compiler.hpp"
#include <stdexcept>

namespace coaster {
namespace {
// The roll fit penalizes velocity, acceleration and jerk while following the
// requested bank. Its linear system couples only three neighbouring controls.
struct BandMatrix {
    std::vector<std::array<double, 4>> rows;
    explicit BandMatrix(size_t n) : rows(n) {}
    double get(size_t i, size_t j) const {
        if (i < j)
            std::swap(i, j);
        return i - j < 4 ? rows[i][i - j] : 0;
    }
    double &at(size_t i, size_t j) {
        if (i < j)
            std::swap(i, j);
        return rows[i][i - j];
    }
    void penalty(size_t first, const std::vector<double> &coefficients, double weight) {
        for (size_t i = 0; i < coefficients.size(); ++i)
            for (size_t j = 0; j <= i; ++j)
                at(first + i, first + j) += weight * coefficients[i] * coefficients[j];
    }
};

std::vector<double> solve(BandMatrix matrix, std::vector<double> rhs,
                          const std::vector<bool> &fixed, const std::vector<double> &values) {
    const size_t n = rhs.size();
    for (size_t i = 0; i < n; ++i)
        if (fixed[i]) {
            for (size_t j = i > 3 ? i - 3 : 0; j < std::min(n, i + 4); ++j)
                if (j != i) {
                    rhs[j] -= matrix.get(i, j) * values[i];
                    matrix.at(i, j) = 0;
                }
            matrix.at(i, i) = 1;
            rhs[i] = values[i];
        }
    BandMatrix lower(n);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = i > 3 ? i - 3 : 0; j <= i; ++j) {
            double value = matrix.get(i, j);
            for (size_t k = i > 3 ? i - 3 : 0; k < j; ++k)
                value -= lower.get(i, k) * lower.get(j, k);
            if (i == j) {
                if (!(value > 0))
                    throw std::runtime_error("Banking programme is not positive definite");
                lower.at(i, i) = std::sqrt(value);
            } else
                lower.at(i, j) = value / lower.get(j, j);
        }
    std::vector<double> result(n);
    for (size_t i = 0; i < n; ++i) {
        double value = rhs[i];
        for (size_t j = i > 3 ? i - 3 : 0; j < i; ++j)
            value -= lower.get(i, j) * result[j];
        result[i] = value / lower.get(i, i);
    }
    for (size_t i = n; i-- > 0;) {
        double value = result[i];
        for (size_t j = i + 1; j < std::min(n, i + 4); ++j)
            value -= lower.get(j, i) * result[j];
        result[i] = value / lower.get(i, i);
    }
    return result;
}

double ease(double u) {
    u = std::clamp(u, 0., 1.);
    return u * u * u * u * (35 + u * (-84 + u * (70 - 20 * u)));
}

double basis(double x, int derivative = 0) {
    if (std::abs(x) >= 3)
        return 0;
    constexpr double choose[]{1, -6, 15, -20, 15, -6, 1};
    double result = 0;
    for (int i = 0; i < 7; ++i)
        result += choose[i] * std::pow(std::max(0., x + 3 - i), 5 - derivative);
    constexpr double factorial[]{120, 24, 6, 2};
    return result / factorial[derivative];
}

using FrameJet = std::array<Vec3, 4>;
constexpr int choose[4][4]{{1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1}};
FrameJet tangentJet(const Knot &k) {
    return {k.tangent, k.curvature, k.third, k.fourth};
}
FrameJet upJet(const Knot &k) {
    return {k.up, k.upFirst, k.upSecond, k.upThird};
}
FrameJet crossJet(const FrameJet &a, const FrameJet &b) {
    FrameJet out{};
    for (int n = 0; n < 4; ++n)
        for (int i = 0; i <= n; ++i)
            out[n] = out[n] + cross(a[i], b[n - i]) * choose[n][i];
    return out;
}
std::array<double, 4> dotJet(const FrameJet &a, const FrameJet &b) {
    std::array<double, 4> out{};
    for (int n = 0; n < 4; ++n)
        for (int i = 0; i <= n; ++i)
            out[n] += dot(a[i], b[n - i]) * choose[n][i];
    return out;
}
detail::AngleJet frameDifference(const Knot &desired, const Knot &base) {
    const auto c = dotJet(upJet(desired), upJet(base)),
               s = dotJet(upJet(desired), crossJet(tangentJet(base), upJet(base)));
    using C = std::complex<double>;
    const C z{c[0], s[0]}, first = C(c[1], s[1]) / z, second = C(c[2], s[2]) / z;
    return {std::arg(z), first.imag(), (second - first * first).imag(),
            (C(c[3], s[3]) / z - 3. * second * first + 2. * first * first * first).imag()};
}
void rotateFrame(Knot &knot, detail::AngleJet angle) {
    const auto up = upJet(knot), right = crossJet(tangentJet(knot), up);
    const auto cosine = detail::cosJet(angle), sine = detail::sinJet(angle);
    const std::array<double, 4> c{cosine.value, cosine.first, cosine.second, cosine.third},
        s{sine.value, sine.first, sine.second, sine.third};
    FrameJet result{};
    for (int n = 0; n < 4; ++n)
        for (int i = 0; i <= n; ++i)
            result[n] = result[n] + (up[i] * c[n - i] + right[i] * s[n - i]) * choose[n][i];
    knot.up = result[0];
    knot.upFirst = result[1];
    knot.upSecond = result[2];
    knot.upThird = result[3];
}

void copyFrame(Knot &destination, const Knot &source) {
    destination.up = source.up;
    destination.upFirst = source.upFirst;
    destination.upSecond = source.upSecond;
    destination.upThird = source.upThird;
    destination.bank = 0;
}

struct TimedMotion {
    double time;
    ReplayMotion motion;
};
TimedMotion motionAtDistance(const std::vector<Frame> &frames, double distance) {
    auto right =
        std::lower_bound(frames.begin(), frames.end(), distance,
                         [](const Frame &frame, double at) { return frame.distance < at; });
    if (right == frames.begin() || right == frames.end()) {
        const auto &frame = right == frames.end() ? frames.back() : *right;
        return {frame.time, {distance, frame.speed, frame.acceleration, frame.accelerationRate}};
    }
    const auto &left = *(right - 1);
    double lo = left.time, hi = right->time;
    double time = lo + (hi - lo) * (distance - left.distance) / (right->distance - left.distance);
    ReplayMotion value;
    for (int i = 0; i < 16; ++i) {
        value = interpolateMotion(left, *right, time);
        const double error = value.distance - distance;
        if (std::abs(error) < 1e-9)
            break;
        if (error > 0)
            hi = time;
        else
            lo = time;
        const double next = time - error / std::max(.01, value.speed);
        time = next > lo && next < hi ? next : (lo + hi) * .5;
    }
    return {time, value};
}

Vec3 transportAcross(const Span &span, Vec3 up) {
    auto derivative = [&](double u, Vec3 value) {
        const auto geometry = sampleSpanGeometry(span, u);
        Vec3 velocity = span.c.back() * double(span.c.size() - 1);
        for (int i = int(span.c.size()) - 2; i >= 1; --i)
            velocity = velocity * u + span.c[size_t(i)] * double(i);
        return geometry.tangent * (-dot(geometry.curvature, value) * norm(velocity));
    };
    const int steps = std::max(1, int(std::ceil(span.length / 2)));
    const double step = 1. / steps;
    for (int i = 0; i < steps; ++i) {
        const double u = double(i) / steps;
        const auto a = derivative(u, up), b = derivative(u + step * .5, up + a * (step * .5));
        const auto c = derivative(u + step * .5, up + b * (step * .5)),
                   d = derivative(u + step, up + c * step);
        up = up + (a + b * 2 + c * 2 + d) * (step / 6);
    }
    const auto tangent = sampleSpanGeometry(span, 1).tangent;
    return unit(up - tangent * dot(up, tangent));
}

void setTransportFrame(Knot &knot, Vec3 up) {
    const double projection = dot(knot.curvature, up);
    const Vec3 first = knot.tangent * (-projection);
    const double firstProjection = dot(knot.third, up) + dot(knot.curvature, first);
    const Vec3 second = knot.curvature * (-projection) - knot.tangent * firstProjection;
    const double secondProjection =
        dot(knot.fourth, up) + 2 * dot(knot.third, first) + dot(knot.curvature, second);
    knot.up = up;
    knot.upFirst = first;
    knot.upSecond = second;
    knot.upThird = knot.third * (-projection) - knot.curvature * (2 * firstProjection) -
                   knot.tangent * secondProjection;
    knot.bank = 0;
}

struct FrameInterval {
    size_t first{}, last{};
    bool inheritFirst{}, inheritLast{};
};

// Each interval owns only the previously unauthored spline frames between
// immutable FVD/hardware ports. No fit is allowed to borrow a neighbouring
// inversion's roll or to overwrite it and repair the source afterwards.
void compileInterval(Design &d, const std::vector<Frame> &frames,
                     const std::vector<double> &routeDistance, FrameInterval interval) {
    const size_t count = interval.last - interval.first + 1;
    if (count < 4)
        throw std::runtime_error("A free frame interval needs at least four canonical knots");
    Track local;
    local.closed =
        d.track.closed && interval.first == 0 && interval.last + 1 == d.track.knots.size();
    local.authoredGeometry = true;
    local.profile = d.track.profile;
    local.knots.assign(d.track.knots.begin() + interval.first,
                       d.track.knots.begin() + interval.last + 1);
    const auto firstPort = local.knots.front(), lastPort = local.knots.back();
    std::vector<TimedMotion> motion(count);
    std::vector<Vec3> reference(count);
    std::vector<double> distance(count), time(count), target(count), load(count),
        uprightAngle(count), positiveLoadAngle(count), positiveBank(count);
    std::vector<bool> hardware(count);
    Vec3 up = local.knots.front().up;
    const double initialTime = motionAtDistance(frames, routeDistance[interval.first]).time;
    const double trainMargin = (d.request.train.cars - 1) * d.request.train.spacing + 1.5;
    for (size_t i = 0; i < count; ++i) {
        const auto &knot = local.knots[i];
        distance[i] = routeDistance[interval.first + i] - routeDistance[interval.first];
        const double routeAt = routeDistance[interval.first + i];
        motion[i] = motionAtDistance(frames, routeAt);
        const double speed = motion[i].motion.speed;
        time[i] = motion[i].time - initialTime;
        if (i)
            up = transportAcross(d.track.spans[interval.first + i - 1], up);
        reference[i] = up = unit(up - knot.tangent * dot(up, knot.tangent));
        Vec3 upright = unit(Vec3{0, 0, 1} - knot.tangent * knot.tangent.z);
        if (norm(upright) < .5)
            upright = up; // A vertical tangent has no gravity-defined bank.
        const Vec3 required = knot.curvature * (speed * speed) + Vec3{0, 0, gravity};
        const double normal = dot(required, upright),
                     lateral = dot(required, cross(knot.tangent, upright));
        load[i] = norm(required - knot.tangent * dot(required, knot.tangent)) / gravity;
        hardware[i] = knot.element == Element::Station || knot.element == Element::Launch ||
                      knot.element == Element::Brake;
        for (const auto &operation : d.operations) {
            const double end =
                operation.kind == DriveKind::Station ? d.track.length : operation.end;
            // Source-owned hardware frames already stop interpolation at the
            // corridor boundary; reserve only the physical train alignment.
            const double margin = trainMargin;
            hardware[i] =
                hardware[i] || (routeAt >= operation.start - margin && routeAt <= end + margin);
        }
        auto angle = [&](Vec3 desired) {
            return std::atan2(dot(desired, cross(knot.tangent, up)), dot(desired, up));
        };
        // Signed load chooses the same upright physical branch through airtime.
        // The regularizer removes the singular direction at zero normal load.
        const double bank = std::atan2(lateral * normal, normal * normal + .09 * gravity * gravity);
        Vec3 desired = hardware[i] ? upright : rotate(upright, knot.tangent, bank);
        if (i == 0 && interval.inheritFirst)
            desired = firstPort.up;
        if (i + 1 == count && interval.inheritLast)
            desired = lastPort.up;
        const double raw = angle(desired);
        target[i] = i ? target[i - 1] + std::remainder(raw - target[i - 1], 2 * pi) : raw;
        uprightAngle[i] = angle(upright);
        positiveBank[i] = std::atan2(lateral, normal);
        positiveLoadAngle[i] = angle(rotate(upright, knot.tangent, positiveBank[i]));
    }
    struct RollPhrase {
        double crestTime;
        std::vector<double> angle;
    };
    std::vector<RollPhrase> phrases;
    for (const auto &source : d.splinePrograms)
        if (source.requestedRoll != 0 && source.lastKnot >= interval.first &&
            source.lastKnot <= interval.last) {
            const size_t crest = source.lastKnot - interval.first;
            if (std::abs(source.requestedRoll) > pi * .5) {
                // An explicitly overbanked source chooses positive rider load.
                // Follow that physical force direction through the crest and
                // recovery; the ordinary signed-load branch stays upright in
                // unrelated airtime. Recover once the load points mostly up.
                const size_t begin = std::max(source.firstKnot, interval.first) - interval.first;
                size_t end = crest;
                while (end + 1 < count && !hardware[end + 1]) {
                    ++end;
                    if (std::abs(positiveBank[end]) < pi * .25)
                        break;
                }
                for (size_t i = begin; i <= end; ++i) {
                    if (hardware[i] || (i == 0 && interval.inheritFirst) ||
                        (i + 1 == count && interval.inheritLast))
                        continue;
                    const double previous = i ? target[i - 1] : target[i];
                    const double desired = load[i] > .3 ? positiveLoadAngle[i] : previous;
                    target[i] = previous + std::remainder(desired - previous, 2 * pi);
                }
                for (size_t i = 1; i < count; ++i)
                    target[i] = target[i - 1] + std::remainder(target[i] - target[i - 1], 2 * pi);
                // The requested bank is a crest cue, not a constant angle to
                // hold throughout a loaded turn. Nearby targets follow its
                // changing force direction rather than fighting that turn.
                auto phrase = target;
                phrase[crest] += std::remainder(
                    uprightAngle[crest] + source.hand * source.requestedRoll - phrase[crest],
                    2 * pi);
                phrases.push_back({time[crest], std::move(phrase)});
            } else {
                phrases.push_back(
                    {time[crest], continuousRollTarget(uprightAngle, target, crest,
                                                       source.hand * source.requestedRoll)});
            }
        }
    if (!(time.back() > 0))
        throw std::runtime_error("Frame interval has no motion duration");
    const size_t controls = std::max<size_t>(4, size_t(std::ceil(time.back() / .1)) + 1);
    const double step = time.back() / (controls - 1);
    std::vector<double> desired(controls), weights(controls), values(controls);
    std::vector<bool> fixed(controls);
    // A zero-spin authored port supplies a stationary bank boundary. The
    // quintic cardinal basis reaches two neighbouring controls at an endpoint;
    // pinning all three makes its first four time derivatives zero there.
    // Pinning only the endpoint value leaves a hidden derivative which the
    // C3 boundary interpolation would otherwise cancel with a reverse roll.
    auto stationaryRoll = [](const Knot &port) {
        const auto right = crossJet(tangentJet(port), upJet(port));
        const double first = dot(port.upFirst, right[0]);
        const double second = dot(port.upSecond, right[0]) + dot(port.upFirst, right[1]);
        const double third = dot(port.upThird, right[0]) + 2 * dot(port.upSecond, right[1]) +
                             dot(port.upFirst, right[2]);
        return std::abs(first) < 1e-9 && std::abs(second) < 1e-9 && std::abs(third) < 1e-9;
    };
    const size_t firstPinned = interval.inheritFirst ? (stationaryRoll(firstPort) ? 3 : 1) : 0;
    const size_t lastPinned = interval.inheritLast ? (stationaryRoll(lastPort) ? 3 : 1) : 0;
    size_t left = 0;
    for (size_t i = 0; i < controls; ++i) {
        const double t = step * i;
        while (left + 2 < count && time[left + 1] < t)
            ++left;
        const double u = (t - time[left]) / (time[left + 1] - time[left]);
        auto sample = [&](const std::vector<double> &v) {
            return v[left] + u * (v[left + 1] - v[left]);
        };
        const double strength = sample(load), balanced = sample(target);
        weights[i] = .4 + strength * strength;
        desired[i] = weights[i] * balanced;
        for (const auto &phrase : phrases) {
            const double weight = 40 * std::pow(1 - ease(std::abs(t - phrase.crestTime) / 2.9), 2);
            desired[i] += weight * sample(phrase.angle);
            weights[i] += weight;
        }
        desired[i] /= weights[i];
        if (hardware[left] || hardware[left + 1]) {
            const double upright =
                uprightAngle[left] +
                u * std::remainder(uprightAngle[left + 1] - uprightAngle[left], 2 * pi);
            values[i] = balanced + std::remainder(upright - balanced, 2 * pi);
            fixed[i] = true;
        }
        if (i < firstPinned || i >= controls - lastPinned) {
            values[i] = i < firstPinned ? target.front() : target.back();
            fixed[i] = true;
        }
    }
    BandMatrix matrix(controls);
    std::vector<double> rhs(controls);
    for (size_t i = 0; i < controls; ++i) {
        matrix.at(i, i) = weights[i];
        rhs[i] = weights[i] * desired[i];
    }
    for (size_t i = 0; i + 1 < controls; ++i)
        matrix.penalty(i, {-1, 1}, .16 / (step * step));
    for (size_t i = 0; i + 2 < controls; ++i)
        matrix.penalty(i, {1, -2, 1}, .08 / std::pow(step, 4));
    for (size_t i = 0; i + 3 < controls; ++i)
        matrix.penalty(i, {-1, 3, -3, 1}, .04 / std::pow(step, 6));
    // Source and hardware boundaries are the only hard controls. A point-mass
    // lateral bound ignores the rotational acceleration of elevated seats and
    // can demand abrupt roll. Independent finite-train replay assesses the
    // actual force/rate histories after this smooth field is compiled.
    auto controlAngle = solve(matrix, rhs, fixed, values);
    // Quadratic smoothing alone keeps following every small bank correction.
    // Total variation instead rewards settled physical attitudes between turns.
    // Because these angles use parallel transport, a constant value means zero
    // rider roll even while the track changes pitch and heading. Load-squared
    // fitting weights retain stronger authority where an angle error would
    // produce a larger lateral force. The same C3 basis and port jets remain.
    constexpr double holdPreference = .35, smoothZeroRate = .002;
    for (int iteration = 0; iteration < 5; ++iteration) {
        auto held = matrix;
        for (size_t i = 0; i + 1 < controls; ++i) {
            const double rate = (controlAngle[i + 1] - controlAngle[i]) / step;
            // Five local reweighted banded solves minimize a smooth absolute
            // roll-rate cost; this never rebuilds geometry or retries a route.
            const double weight = holdPreference / (step * step * std::hypot(rate, smoothZeroRate));
            held.penalty(i, {-1, 1}, weight);
        }
        controlAngle = solve(std::move(held), rhs, fixed, values);
    }
    for (size_t i = 0; i < count; ++i) {
        const double x = time[i] / step;
        const int center = int(std::floor(x));
        std::array<double, 4> angle{};
        for (int derivative = 0; derivative < 4; ++derivative) {
            for (int j = center - 2; j <= center + 3; ++j)
                angle[size_t(derivative)] +=
                    controlAngle[size_t(std::clamp(j, 0, int(controls) - 1))] *
                    basis(x - j, derivative);
            angle[size_t(derivative)] /= std::pow(step, derivative);
        }
        const auto &dynamics = motion[i].motion;
        const double speed = std::max(5., dynamics.speed), acceleration = dynamics.acceleration,
                     jerk = dynamics.jerk;
        // Exact chain rule from the time-domain bank field to arc derivatives.
        // These are independent of canonical knot spacing.
        const detail::AngleJet spatial{
            angle[0], angle[1] / speed,
            angle[2] / (speed * speed) - angle[1] * acceleration / std::pow(speed, 3),
            angle[3] / std::pow(speed, 3) - 3 * angle[2] * acceleration / std::pow(speed, 4) +
                angle[1] * (3 * acceleration * acceleration / std::pow(speed, 5) -
                            jerk / std::pow(speed, 4))};
        setTransportFrame(local.knots[i], reference[i]);
        rotateFrame(local.knots[i], spatial);
    }

    // Boundary jets are interpolation constraints of this interval, not a
    // restoration pass over somebody else's frame. Blend their residual with
    // a C3 Hermite basis; no position or source-owned knot is ever changed.
    const double length = distance.back();
    const double firstWidth = std::min(
        length * .5, std::max(20., 1.2 * replayValueAt(frames, routeDistance[interval.first])));
    const double lastWidth = std::min(
        length * .5, std::max(20., 1.2 * replayValueAt(frames, routeDistance[interval.last])));
    const auto firstBoundary = detail::anglePolynomial(
        interval.inheritFirst ? frameDifference(firstPort, local.knots.front())
                              : detail::AngleJet{},
        {}, firstWidth);
    const auto lastBoundary = detail::anglePolynomial(
        {},
        interval.inheritLast ? frameDifference(lastPort, local.knots.back()) : detail::AngleJet{},
        lastWidth);
    const size_t begin = interval.inheritFirst ? 1 : 0,
                 end = count - (interval.inheritLast ? 1 : 0);
    for (size_t i = begin; i < end; ++i) {
        if (interval.inheritFirst && distance[i] < firstWidth)
            rotateFrame(local.knots[i], detail::angleAt(firstBoundary, distance[i], firstWidth));
        else if (interval.inheritLast && length - distance[i] < lastWidth)
            rotateFrame(local.knots[i],
                        detail::angleAt(lastBoundary, lastWidth - length + distance[i], lastWidth));
        copyFrame(d.track.knots[interval.first + i], local.knots[i]);
    }
}
} // namespace

std::vector<double> continuousRollTarget(const std::vector<double> &upright,
                                         const std::vector<double> &reference, size_t crest,
                                         double bank) {
    if (upright.size() != reference.size() || crest >= upright.size())
        throw std::invalid_argument("Invalid roll phrase domain");
    std::vector<double> result(upright.size());
    result[crest] =
        reference[crest] + std::remainder(upright[crest] + bank - reference[crest], 2 * pi);
    for (size_t i = crest; i-- > 0;)
        result[i] = result[i + 1] + std::remainder(upright[i] + bank - result[i + 1], 2 * pi);
    for (size_t i = crest + 1; i < result.size(); ++i)
        result[i] = result[i - 1] + std::remainder(upright[i] + bank - result[i - 1], 2 * pi);
    return result;
}

void authorBanking(Design &d, const std::vector<Frame> &frames) {
    if (frames.empty())
        throw std::invalid_argument("Frame compilation requires a reached motion replay");
    const size_t count = d.track.knots.size();
    if (count < 4 || d.track.spans.size() + 1 != count)
        throw std::invalid_argument("Frame compilation requires rebuilt geometry");
    std::vector<bool> owned(count);
    for (const auto &source : d.forcePrograms) {
        if (source.firstKnot >= count || source.sourceDistances.size() > count - source.firstKnot)
            throw std::invalid_argument("Force frame owner is outside the route");
        for (size_t i = source.firstKnot; i < source.firstKnot + source.sourceDistances.size(); ++i)
            owned[i] = true;
    }
    std::vector<double> distance(count);
    for (size_t i = 0; i < count; ++i) {
        distance[i] = i + 1 == count ? d.track.length : d.track.spans[i].start;
        auto &knot = d.track.knots[i];
        const bool straight = norm(knot.curvature) + norm(knot.third) + norm(knot.fourth) < 1e-12;
        const bool hardware = knot.element == Element::Station || knot.element == Element::Launch ||
                              knot.element == Element::Brake;
        if (!owned[i] && straight && hardware) {
            knot.up = unit(Vec3{0, 0, 1} - knot.tangent * knot.tangent.z);
            knot.upFirst = knot.upSecond = knot.upThird = {};
            knot.bank = 0;
            owned[i] = true;
        }
    }
    std::vector<FrameInterval> intervals;
    for (size_t i = 0; i < count;) {
        if (owned[i]) {
            ++i;
            continue;
        }
        const size_t begin = i;
        while (i < count && !owned[i])
            ++i;
        intervals.push_back(
            {begin ? begin - 1 : 0, i < count ? i : count - 1, begin > 0, i < count});
    }
    for (const auto interval : intervals)
        compileInterval(d, frames, distance, interval);
    if (d.track.closed)
        d.track.knots.back() = d.track.knots.front();
    d.track.authoredFrame = true;
    d.track.rebuild();
}
} // namespace coaster
