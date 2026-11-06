#include "authoring.hpp"
#include "motion_program.hpp"
#include "curve_compiler.hpp"
#include <stdexcept>
#include <sstream>

namespace coaster {
using detail::MotionJet;
MotionBuilder::MotionBuilder(Design &design, Cancel stop)
    : d(design), req(design.request), cancel(std::move(stop)), datum(req.limits.minClearance + 4.5),
      drag(.5 * req.train.airDensity * req.train.dragCdA / (req.train.cars * req.train.carMass)),
      cursor(detail::planarJet({0, 0, datum}, 0, 0)) {
    d.track.authoredGeometry = true;
    d.track.knots.push_back(
        {cursor.position, cursor.tangent, cursor.curvature, {0, 0, 1}, 0, Element::Station});
}
FvdEntry MotionBuilder::fvdEntry(double speed, FvdDriveJet drive) const {
    if (d.track.knots.empty())
        throw std::runtime_error("FVD entry requires a physical port");
    const auto &knot = d.track.knots.back();
    const size_t index = d.track.knots.size() - 1;
    const bool owned =
        std::any_of(d.forcePrograms.begin(), d.forcePrograms.end(), [&](const auto &source) {
            return index >= source.firstKnot &&
                   index - source.firstKnot < source.sourceDistances.size();
        });
    if (!owned && !d.track.authoredFrame &&
        norm(knot.curvature) + norm(knot.third) + norm(knot.fourth) > 1e-10)
        throw std::runtime_error(
            "FVD entry needs the spline's finalized analytic physical frame, not its placeholder");
    return makeFvdEntry(knot, speed, gravity * req.train.rollingResistance, drag, drive);
}
double MotionBuilder::coastEnergy(double squaredSpeed, double rise, double distance) const {
    if (distance <= 0)
        return squaredSpeed;
    // Exact constant-grade solution of d(v²)/ds = -2g(grade+rolling)-2drag*v².
    // This planning estimate does not depend on a fixed dense sampling interval;
    // the separate finite-train energy cursor supplies authoritative source ports.
    const double exponent = 2 * drag * distance;
    const double response = exponent == 0 ? 1 : -std::expm1(-exponent) / exponent;
    return squaredSpeed * std::exp(-exponent) -
           2 * gravity * (rise + req.train.rollingResistance * distance) * response;
}
TrackSample MotionBuilder::sampleGeometry(double distance) const {
    if (d.track.spans.empty()) {
        const auto &knot = d.track.knots.front();
        return {knot.position, knot.tangent, knot.curvature, knot.up, cross(knot.tangent, knot.up),
                knot.element};
    }
    // A source under construction is an open prefix, regardless of eventual closure.
    const double at = std::clamp(distance, 0., d.track.length);
    if (at == d.track.length)
        return sampleSpanGeometry(d.track.spans.back(), 1,
                                  d.track.knots[d.track.knots.size() - 2].element);
    const auto location = d.track.locate(at);
    return sampleSpanGeometry(d.track.spans[location.span], location.parameter,
                              d.track.knots[location.span].element);
}
void MotionBuilder::append(const MotionJet &q, Element e, Vec3 up, double bank) {
    if (cancel && cancel())
        throw std::runtime_error("CANCELLED");
    up = unit(up - q.tangent * dot(q.tangent, up));
    if (norm(up) < .5) {
        const Vec3 axis = std::abs(q.tangent.z) > .9 ? Vec3{1, 0, 0} : Vec3{0, 0, 1};
        up = unit(axis - q.tangent * dot(axis, q.tangent));
    }
    double distance = 0;
    if (norm(q.position - d.track.knots.back().position) > 1e-6) {
        Knot knot{q.position, q.tangent, q.curvature, up, bank, e, q.third, q.fourth};
        auto span = compileCanonicalSpan(d.track.knots.back(), knot);
        distance = span.length;
        span.start = d.track.length;
        d.track.length += span.length;
        // Element identity belongs to the outgoing span, including its first
        // boundary; coarse source spans must not inherit the previous element.
        d.track.knots.back().element = e;
        d.track.spans.push_back(std::move(span));
        d.track.knots.push_back(knot);
    }
    nominalSpeed = std::sqrt(std::max(
        0., coastEnergy(nominalSpeed * nominalSpeed, q.position.z - cursor.position.z, distance)));
    cursor = q;
}

MotionBuilder::Range MotionBuilder::force(const FvdResult &result, const FvdRequest &program,
                                          Element element, const char *name, Vec3 origin,
                                          double heading, double begin, double end) {
    if (!result.report.valid() || !result.assessment.passed)
        throw std::runtime_error(std::string(name) + ": force source failed its canonical replay");
    if (end < 0)
        end = result.track.length;
    if (!std::isfinite(begin) || !std::isfinite(end) || begin < 0 || end <= begin ||
        end > result.track.length)
        throw std::runtime_error(std::string(name) +
                                 ": force source interval is outside the authored track");
    auto rotate = [&](Vec3 p) {
        return Vec3{p.x * std::cos(heading) - p.y * std::sin(heading),
                    p.x * std::sin(heading) + p.y * std::cos(heading), p.z};
    };
    auto pose = [&](double distance) {
        auto q = detail::jet(sampleKinematics(result.track, distance));
        q.position = origin + rotate(q.position);
        q.tangent = rotate(q.tangent);
        q.curvature = rotate(q.curvature);
        q.third = rotate(q.third);
        q.fourth = rotate(q.fourth);
        return q;
    };
    const auto first = pose(begin);
    if (norm(first.position - cursor.position) > 1e-7 ||
        norm(first.tangent - cursor.tangent) > 1e-7 ||
        norm(first.curvature - cursor.curvature) > 1e-7 ||
        norm(first.third - cursor.third) > 1e-7 || norm(first.fourth - cursor.fourth) > 1e-7)
        throw std::runtime_error(std::string(name) +
                                 ": force source must inherit the complete live geometry jet");
    const size_t firstKnot = d.track.knots.size() - 1;
    const bool frameOwned =
        d.track.authoredFrame ||
        std::any_of(d.forcePrograms.begin(), d.forcePrograms.end(), [&](const auto &source) {
            return firstKnot >= source.firstKnot &&
                   firstKnot - source.firstKnot < source.sourceDistances.size();
        });
    if (frameOwned) {
        const auto incoming = sampleKinematics(result.track, begin);
        const auto &previous = d.track.knots.back();
        if (norm(rotate(incoming.sample.up) - previous.up) > 1e-7 ||
            norm(rotate(incoming.upS) - previous.upFirst) > 1e-7 ||
            norm(rotate(incoming.upSS) - previous.upSecond) > 1e-7 ||
            norm(rotate(incoming.upSSS) - previous.upThird) > 1e-7)
            throw std::runtime_error(
                std::string(name) +
                ": force source must inherit the complete live physical frame jet");
    }
    // An unfinalized neighboring spline may adopt this boundary frame during
    // its later analytic banking pass. An existing FVD owner may never do so.
    auto retainFrame = [&](Knot &knot, double distance) {
        const auto q = sampleKinematics(result.track, distance);
        knot.up = rotate(q.sample.up);
        knot.bank = 0;
        knot.upFirst = rotate(q.upS);
        knot.upSecond = rotate(q.upSS);
        knot.upThird = rotate(q.upSSS);
    };
    retainFrame(d.track.knots.back(), begin);
    ForceAuthoring source;
    source.name = name;
    source.program = program;
    source.origin = origin;
    source.heading = heading;
    source.firstKnot = firstKnot;
    source.sourceDistances.push_back(begin);
    // Keep force-control boundaries as canonical knots. A span across a
    // change in the next derivative cannot preserve the source's interior jerk.
    std::vector<double> boundaries{begin}, times;
    for (const auto &control : program.controls)
        times.push_back(control.time);
    for (const auto &phase : program.twists) {
        times.push_back(phase.begin);
        times.push_back(phase.end);
    }
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());
    for (double time : times) {
        const auto at =
            std::lower_bound(result.samples.begin(), result.samples.end(), time,
                             [](const auto &sample, double value) { return sample.time < value; });
        if (at == result.samples.end() || std::abs(at->time - time) > 1e-9)
            throw std::runtime_error("Force source lost an authored phase boundary");
        const size_t index = size_t(at - result.samples.begin());
        const double distance = index < result.track.spans.size() ? result.track.spans[index].start
                                                                  : result.track.length;
        if (distance > begin + 1e-6 && distance < end - 1e-6)
            boundaries.push_back(distance);
    }
    boundaries.push_back(end);
    // Crests and valleys are semantic source boundaries, not whichever sparse
    // sample happens to be highest. Retain their exact zero-grade locations so
    // section labels and terrain anchors remain independent of tessellation.
    size_t previous = 0;
    int previousSign = 0;
    for (size_t i = 0; i < result.track.knots.size(); ++i) {
        const double grade = result.track.knots[i].tangent.z;
        const int sign = grade > 1e-8 ? 1 : grade < -1e-8 ? -1 : 0;
        if (!sign)
            continue;
        if (previousSign && sign != previousSign) {
            double low = previous < result.track.spans.size() ? result.track.spans[previous].start
                                                              : result.track.length;
            double high =
                i < result.track.spans.size() ? result.track.spans[i].start : result.track.length;
            for (int iteration = 0; iteration < 40; ++iteration) {
                const double middle = (low + high) * .5;
                if (result.track.sample(middle).tangent.z * previousSign > 0)
                    low = middle;
                else
                    high = middle;
            }
            const double root = (low + high) * .5;
            if (root > begin + 1e-6 && root < end - 1e-6)
                boundaries.push_back(root);
        }
        previous = i;
        previousSign = sign;
    }
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
                                 [](double a, double b) { return std::abs(a - b) < 1e-7; }),
                     boundaries.end());
    auto curve = [&](double distance) {
        const auto physical = sampleKinematics(result.track, distance);
        Knot knot{origin + rotate(physical.sample.position),
                  rotate(physical.sample.tangent),
                  rotate(physical.sample.curvature),
                  rotate(physical.sample.up),
                  0,
                  element,
                  rotate(physical.curvatureS),
                  rotate(physical.curvatureSS),
                  rotate(physical.upS),
                  rotate(physical.upSS),
                  rotate(physical.upSSS)};
        return SourceCurveSample{knot, true};
    };
    for (size_t interval = 1; interval < boundaries.size(); ++interval) {
        const auto distances = compileCurveDistances(curve, boundaries[interval - 1],
                                                     boundaries[interval], {}, cancel);
        for (size_t i = 1; i < distances.size(); ++i) {
            const double distance = distances[i];
            const auto sample = curve(distance);
            const auto &knot = sample.knot;
            append({knot.position, knot.tangent, knot.curvature, knot.third, knot.fourth}, element,
                   knot.up);
            d.track.knots.back().upFirst = knot.upFirst;
            d.track.knots.back().upSecond = knot.upSecond;
            d.track.knots.back().upThird = knot.upThird;
            source.sourceDistances.push_back(distance);
        }
    }
    d.forcePrograms.push_back(std::move(source));
    return {firstKnot, d.track.knots.size() - 1};
}

MotionBuilder::Range MotionBuilder::programme(const MotionProgram &program, Element element,
                                              const char *name, double releaseBank) {
    const size_t begin = d.track.knots.size() - 1;
    const auto start = cursor;
    const double entrySpeed = nominalSpeed;
    if (!std::isfinite(program.length) || program.length <= 0)
        throw std::runtime_error(std::string(name) + ": motion length must be finite and positive");
    const auto inherited = program.direction(0);
    if (norm(inherited.tangent - start.tangent) > 1e-7 ||
        norm(inherited.curvature - start.curvature) > 1e-7 ||
        norm(inherited.third - start.third) > 1e-7 || norm(inherited.fourth - start.fourth) > 1e-7)
        throw std::runtime_error(std::string(name) +
                                 ": motion programme must inherit the complete live geometry jet");
    const auto first = detail::directionAngles(start), last = detail::directionAngles(program.end);
    // Integrate each analytic B-spline interval once. Arbitrary adaptive probes
    // then need one short quadrature, not a whole-prefix integration.
    constexpr int sourceIntervals = MotionProgram::controlCount - MotionProgram::degree;
    std::array<double, sourceIntervals + 1> boundaries{};
    std::array<Vec3, sourceIntervals + 1> positions{};
    for (int i = 1; i <= sourceIntervals; ++i) {
        boundaries[i] = program.length * .5 * (1 - std::cos(pi * i / sourceIntervals));
        positions[i] =
            positions[i - 1] +
            detail::integrateDirection([&](double at) { return program.direction(at).tangent; },
                                       boundaries[i - 1], boundaries[i]);
    }
    auto curve = [&](double distance) {
        auto q = program.direction(distance);
        const auto found = std::upper_bound(boundaries.begin(), boundaries.end(), distance);
        const size_t part =
            std::min<size_t>(sourceIntervals - 1, size_t(found - boundaries.begin() - 1));
        q.position =
            start.position + positions[part] +
            detail::integrateDirection([&](double at) { return program.direction(at).tangent; },
                                       boundaries[part], distance);
        if (distance == program.length)
            q = program.end;
        Vec3 up = unit(Vec3{0, 0, 1} - q.tangent * q.tangent.z);
        return SourceCurveSample{
            {q.position, q.tangent, q.curvature, up, 0, element, q.third, q.fourth}, false};
    };
    const size_t count = compileUniformCurveIntervals(curve, program.length, {}, cancel);
    for (size_t i = 1; i <= count; ++i) {
        const auto sample = curve(program.length * i / count);
        const auto &q = sample.knot;
        append({q.position, q.tangent, q.curvature, q.third, q.fourth}, element, q.up);
    }
    d.splinePrograms.push_back({name, begin, d.track.knots.size() - 1, start.position,
                                program.length, entrySpeed, gravity * req.train.rollingResistance,
                                drag, 1, releaseBank, program.pitch, program.heading});
    const double initial = std::abs(first[0].value) > .002 ? first[0].value : first[0].first;
    const double terminal = std::abs(last[0].value) > .002 ? last[0].value : -last[0].first;
    const bool planar = std::abs(dot(program.end.position - start.position,
                                     unit(Vec3{-start.tangent.y, start.tangent.x, 0}))) < 1e-8 &&
                        std::abs(std::remainder(last[1].value - first[1].value, 2 * pi)) +
                                std::abs(first[1].first) + std::abs(first[1].second) +
                                std::abs(first[1].third) + std::abs(last[1].first) +
                                std::abs(last[1].second) + std::abs(last[1].third) <
                            1e-10;
    modules.push_back(
        {begin, d.track.knots.size() - 1, name, initial * terminal < 0 ? 1 : 0, planar});
    return {begin, d.track.knots.size() - 1};
}

MotionBuilder::Range MotionBuilder::pitchToHeight(double height, detail::AngleJet endPitch,
                                                  Element element, const char *name) {
    return heightCurve(height, endPitch, {std::atan2(cursor.tangent.y, cursor.tangent.x), 0, 0, 0},
                       element, name);
}
MotionProgram MotionBuilder::heightMotion(double height, detail::AngleJet endPitch,
                                          detail::AngleJet endHeading) const {
    const auto start = cursor;
    const auto angles = detail::directionAngles(start);
    const double target = height - start.position.z;
    if (std::abs(target) < 1e-3)
        throw std::runtime_error("Height programme needs a real elevation change");
    endHeading.value = angles[1].value + std::remainder(endHeading.value - angles[1].value, 2 * pi);
    auto polynomial = [&](double length) {
        return detail::anglePolynomial(angles[0], endPitch, length);
    };
    auto elevation = [&](double length) {
        const auto p = polynomial(length);
        double z = 0;
        for (int i = 0; i < 16; ++i)
            z += detail::integrateDirection(
                     [&](double at) {
                         return Vec3{0, 0, std::sin(detail::angleAt(p, at, length).value)};
                     },
                     length * i / 16, length * (i + 1) / 16)
                     .z;
        return z;
    };
    const double sign = target > 0 ? 1. : -1.;
    double lo = 1, hi = 10;
    while (hi < 800 && (elevation(hi) - target) * sign < 0) {
        lo = hi;
        hi += 10;
    }
    if (hi >= 800)
        throw std::runtime_error(std::string("Height motion cannot reach the requested height"));
    for (int i = 0; i < 45; ++i) {
        double mid = (lo + hi) * .5;
        if ((elevation(mid) - target) * sign < 0)
            lo = mid;
        else
            hi = mid;
    }
    const double length = (lo + hi) * .5;
    const auto p = polynomialMotion(start, polynomial(length),
                                    detail::anglePolynomial(angles[1], endHeading, length), length);
    if (std::abs(p.end.position.z - height) > 1e-7)
        throw std::runtime_error(
            "Polynomial height solve failed its independent displacement check");
    return p;
}
MotionBuilder::Range MotionBuilder::heightCurve(double height, detail::AngleJet endPitch,
                                                detail::AngleJet endHeading, Element element,
                                                const char *name) {
    return programme(heightMotion(height, endPitch, endHeading), element, name);
}

MotionBuilder::Range MotionBuilder::line(double length, Element element, const char *name) {
    if (norm(cursor.curvature) + norm(cursor.third) + norm(cursor.fourth) > 1e-7)
        throw std::runtime_error("Hardware corridor must inherit aligned derivatives");
    if (element == Element::Station || element == Element::Launch || element == Element::Brake) {
        const size_t index = d.track.knots.size() - 1;
        const auto &port = d.track.knots.back();
        const bool physical =
            d.track.authoredFrame ||
            std::any_of(d.forcePrograms.begin(), d.forcePrograms.end(), [&](const auto &source) {
                return index >= source.firstKnot &&
                       index - source.firstKnot < source.sourceDistances.size();
            });
        const Vec3 upright = unit(Vec3{0, 0, 1} - port.tangent * port.tangent.z);
        if (physical && norm(port.up - upright) + norm(port.upFirst) + norm(port.upSecond) +
                                norm(port.upThird) >
                            1e-7)
            throw std::runtime_error("Hardware corridor must inherit an upright stationary "
                                     "physical frame; release the bank in its authored source");
    }
    const size_t begin = d.track.knots.size() - 1;
    const auto start = cursor;
    const int count = std::max(3, int(std::ceil(length / 60)));
    for (int i = 1; i <= count; ++i) {
        auto q = start;
        q.position = start.position + start.tangent * (length * i / count);
        q.curvature = q.third = q.fourth = {};
        append(q, element);
    }
    modules.push_back({begin, d.track.knots.size() - 1, name});
    return std::pair{begin, d.track.knots.size() - 1};
}

void MotionBuilder::drive(double length, DriveKind kind, double speed, double acceleration,
                          const char *name) {
    auto range = line(length, kind == DriveKind::Brake ? Element::Brake : Element::Launch, name);
    motors.push_back({range.first, range.second, kind, speed, acceleration});
    nominalSpeed = speed - .3;
}

void MotionBuilder::driveGraded(double endHeight, double endGrade, double speed,
                                double acceleration, const char *name) {
    const size_t begin = d.track.knots.size() - 1, firstModule = modules.size();
    line(std::max(40., speed * 1.8), Element::Launch, "powered-first-grade");
    const auto angles = detail::directionAngles(cursor);
    const double blendLength = speed * 1.5;
    const auto pitch = detail::anglePolynomial(angles[0], {endGrade, 0, 0, 0}, blendLength);
    programme(polynomialMotion(cursor, pitch, {angles[1].value, 0, 0, 0, 0, 0, 0, 0}, blendLength),
              Element::Launch, "powered-grade-blend");
    line((endHeight - cursor.position.z) / std::sin(endGrade), Element::Launch,
         "powered-second-grade");
    modules.resize(firstModule);
    modules.push_back({begin, d.track.knots.size() - 1, name, 0, true});
    motors.push_back({begin, d.track.knots.size() - 1, DriveKind::Boost, speed, acceleration});
    nominalSpeed = speed - .3;
}

double MotionBuilder::crestCurvature(double rise, double length, double targetG) {
    const double initial = nominalSpeed * nominalSpeed, top = initial - 2 * gravity * rise;
    const double estimate =
        top -
        2 * (gravity * req.train.rollingResistance + drag * (initial + std::max(0., top)) * .5) *
            length;
    if (estimate < 100) {
        std::ostringstream error;
        error << "Coasting hill has insufficient estimated crest energy: entry=" << nominalSpeed
              << ", rise=" << rise << ", length=" << length << ", exitEnergy=" << estimate;
        throw std::runtime_error(error.str());
    }
    return (targetG - 1) * gravity / estimate;
}

double replayValueAt(const std::vector<Frame> &frames, double distance, bool time) {
    if (frames.empty())
        throw std::runtime_error("Authoring feedback requires an actual replay");
    auto right = std::lower_bound(frames.begin(), frames.end(), distance,
                                  [](const Frame &f, double s) { return f.distance < s; });
    if (right == frames.begin())
        return time ? right->time : right->speed;
    if (right == frames.end())
        return time ? frames.back().time : frames.back().speed;
    const auto &left = *(right - 1);
    const double h = right->distance - left.distance, u = (distance - left.distance) / h;
    if (time)
        return left.time + u * (right->time - left.time);
    // Energy has d(v^2)/ds=2a. Preserve its physical derivative rather than
    // feeding 60 Hz linear-speed corners into a third-order orientation field.
    const double c0 = left.speed * left.speed, c1 = 2 * left.acceleration * h;
    const double c2 = left.speed > .1 ? left.accelerationRate / left.speed * h * h : 0;
    const double p = right->speed * right->speed - c0 - c1 - c2,
                 v = 2 * right->acceleration * h - c1 - 2 * c2;
    const double a =
        (right->speed > .1 ? 2 * right->accelerationRate / right->speed * h * h : 0) - 2 * c2;
    return std::sqrt(std::max(
        0.,
        c0 + u * (c1 + u * (c2 + u * (10 * p - 4 * v + a * .5 +
                                      u * (-15 * p + 7 * v - a + u * (6 * p - 3 * v + a * .5)))))));
}
} // namespace coaster
