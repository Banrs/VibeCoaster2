#include "route_closure.hpp"
#include <optional>
#include <stdexcept>
#include <sstream>

namespace coaster {
namespace {
double cross2(Vec3 a, Vec3 b) {
    return a.x * b.y - a.y * b.x;
}
Vec3 direction(double angle) {
    return {std::cos(angle), std::sin(angle), 0};
}
struct CurvatureTurn {
    double curvature{}, ramp{}, hold{};
    double length() const {
        return 2 * ramp + hold;
    }
};
CurvatureTurn turnShape(double angle, double radius, double speed) {
    if (std::abs(angle) < 1e-12)
        return {};
    // Ramp curvature rather than stretching a single yaw ease across the whole
    // turn. A real constant-radius middle can turn compactly while each bank
    // entry/release retains continuous first and second curvature derivatives.
    const double ramp = 1.4 * speed * std::sqrt(std::min(1., std::abs(angle)));
    const double curvature = std::min(1 / radius, std::abs(angle) / ramp);
    return {std::copysign(curvature, angle), ramp,
            std::max(0., std::abs(angle) / curvature - ramp)};
}
double turnLength(double angle, double radius, double speed) {
    return turnShape(angle, radius, speed).length();
}
Vec3 turnDisplacement(double begin, double end, double radius, double speed) {
    const auto shape = turnShape(end - begin, radius, speed);
    if (shape.length() == 0)
        return {};
    auto integratedEase = [](double u) { return u * u * u * u * (2.5 + u * (-3 + u)); };
    Vec3 result{};
    auto integrate = [&](auto heading, double length) {
        for (int i = 0; i < 4; ++i)
            result =
                result + detail::integrateDirection([&](double u) { return direction(heading(u)); },
                                                    double(i) / 4, double(i + 1) / 4) *
                             length;
    };
    integrate([&](double u) { return begin + shape.curvature * shape.ramp * integratedEase(u); },
              shape.ramp);
    const double arcBegin = begin + .5 * shape.curvature * shape.ramp;
    if (shape.hold > 0)
        integrate([&](double u) { return arcBegin + shape.curvature * shape.hold * u; },
                  shape.hold);
    const double releaseBegin = arcBegin + shape.curvature * shape.hold;
    integrate(
        [&](double u) {
            return releaseBegin + shape.curvature * shape.ramp * (u - integratedEase(u));
        },
        shape.ramp);
    return result;
}
struct Approach {
    double middle{}, end{}, radius{}, straight{}, cost{};
};
struct TurnPair {
    std::vector<MotionProgram> first, second;
    detail::MotionJet end;
};
TurnPair exactApproach(const detail::MotionJet &start, const Approach &plan, double speed) {
    TurnPair result;
    auto cursor = start;
    double unwrapped = detail::directionAngles(start)[1].value;
    auto turn = [&](double heading) {
        std::vector<MotionProgram> programs;
        const auto shape = turnShape(heading - unwrapped, plan.radius, speed);
        if (shape.length() < 1e-8)
            return programs;
        const double begin = unwrapped;
        auto append = [&](double length, double yaw, double curvature) {
            auto angles = detail::directionAngles(cursor);
            angles[1].value = unwrapped;
            auto program = polynomialMotion(
                cursor, detail::anglePolynomial(angles[0], {0, 0, 0, 0}, length),
                detail::anglePolynomial(angles[1], {yaw, curvature, 0, 0}, length), length);
            cursor = program.end;
            unwrapped = yaw;
            programs.push_back(std::move(program));
        };
        append(shape.ramp, begin + .5 * shape.curvature * shape.ramp, shape.curvature);
        if (shape.hold > 1e-8)
            append(shape.hold, unwrapped + shape.curvature * shape.hold, shape.curvature);
        append(shape.ramp, heading, 0);
        return programs;
    };
    result.first = turn(plan.middle);
    cursor.position = cursor.position + cursor.tangent * plan.straight;
    result.second = turn(plan.end);
    result.end = cursor;
    return result;
}
} // namespace

void closeRouteApproach(MotionBuilder &motion, Vec3 endPosition, double endHeading,
                        const char *name, Cancel cancel) {
    const auto start = motion.cursor;
    const auto incoming = detail::directionAngles(start);
    if (!finite(endPosition) || !std::isfinite(endHeading) ||
        std::abs(start.position.z - endPosition.z) > 1e-7 || std::abs(start.tangent.z) > 1e-8 ||
        norm(start.curvature) + norm(start.third) + norm(start.fourth) > 1e-8)
        throw std::runtime_error(std::string(name) + ": approach requires an inherited level port");
    const Vec3 target = endPosition - start.position;
    const double speed = motion.nominalSpeed;
    if (!std::isfinite(speed) || speed <= 0)
        throw std::runtime_error("Approach needs positive entry energy");
    const double begin = incoming[1].value;
    endHeading = begin + std::remainder(endHeading - begin, 2 * pi);
    if (std::abs(std::remainder(endHeading - begin, 2 * pi)) < 1e-10 &&
        std::abs(cross2(target, start.tangent)) < 1e-8 && dot(target, start.tangent) > 0) {
        motion.line(dot(target, start.tangent), Element::Turn, name);
        return;
    }
    // Coordinated 3.2 g is a construction target, not a validation waiver.
    // The final finite-train replay measures all seats and force durations.
    const double minimumRadius =
        std::max(12., speed * speed / (gravity * std::sqrt(3.2 * 3.2 - 1)));
    std::optional<Approach> selected;
    for (double scale : {1., 1.2, 1.5, 2., 3.}) {
        const double radius = minimumRadius * scale;
        for (int winding : {0, -1, 1}) {
            const double finish = endHeading + winding * 2 * pi;
            auto residual = [&](double middle) {
                return cross2(target - turnDisplacement(begin, middle, radius, speed) -
                                  turnDisplacement(middle, finish, radius, speed),
                              direction(middle));
            };
            auto consider = [&](double middle) {
                const double first = middle - begin, last = finish - middle;
                if (std::abs(first) > 1.5 * pi || std::abs(last) > 1.5 * pi ||
                    std::abs(first) + std::abs(last) > 2 * pi + 1e-8)
                    return;
                const auto rest = target - turnDisplacement(begin, middle, radius, speed) -
                                  turnDisplacement(middle, finish, radius, speed);
                const double straight = dot(rest, direction(middle));
                if (straight < 0 || std::abs(cross2(rest, direction(middle))) > 1e-7)
                    return;
                const double length =
                    turnLength(first, radius, speed) + straight + turnLength(last, radius, speed);
                if (!selected || length < selected->cost)
                    selected = Approach{middle, finish, radius, straight, length};
            };
            const double lower = std::max(begin - 1.5 * pi, finish - 1.5 * pi);
            const double upper = std::min(begin + 1.5 * pi, finish + 1.5 * pi);
            constexpr int samples = 256;
            double left = lower, previous = residual(left);
            for (int i = 1; i <= samples; ++i) {
                if (cancel && cancel())
                    throw std::runtime_error("Approach planning cancelled");
                const double right = std::lerp(lower, upper, double(i) / samples),
                             value = residual(right);
                if (previous * value <= 0) {
                    double lo = left, hi = right, fl = previous;
                    for (int step = 0; step < 54; ++step) {
                        const double middle = (lo + hi) * .5, fm = residual(middle);
                        if (fl * fm <= 0)
                            hi = middle;
                        else {
                            lo = middle;
                            fl = fm;
                        }
                    }
                    consider((lo + hi) * .5);
                }
                left = right;
                previous = value;
            }
            consider(begin);
            consider(finish);
        }
    }
    if (!selected) {
        std::ostringstream detail;
        detail << name << ": no forward C3 approach from (" << start.position.x << ','
               << start.position.y << ") heading " << begin << " at " << speed
               << " m/s fits station port (" << endPosition.x << ',' << endPosition.y << ')';
        throw std::runtime_error(detail.str());
    }

    // Correct quadrature and inherited-jet roundoff in the two global layout
    // variables. No final span or canonical endpoint is moved independently.
    auto plan = *selected;
    for (int iteration = 0; iteration < 4; ++iteration) {
        const auto built = exactApproach(start, plan, speed);
        const auto error = built.end.position - endPosition;
        if (std::hypot(error.x, error.y) < 1e-10)
            break;
        auto perturbed = plan;
        perturbed.middle += 1e-6;
        const auto derivative =
            (exactApproach(start, perturbed, speed).end.position - built.end.position) / 1e-6;
        const auto straight = direction(plan.middle);
        const double determinant = cross2(derivative, straight);
        if (std::abs(determinant) < 1e-9)
            break;
        plan.middle += cross2(error * -1, straight) / determinant;
        plan.straight += cross2(derivative, error * -1) / determinant;
    }
    const auto compiled = exactApproach(start, plan, speed);
    if (plan.straight < 0 || norm(compiled.end.position - endPosition) > 1e-7)
        throw std::runtime_error(std::string(name) +
                                 ": exact approach integration missed the station port");
    const std::string label = name;
    for (size_t i = 0; i < compiled.first.size(); ++i)
        motion.programme(compiled.first[i], Element::Turn,
                         (label + "-entry-turn-" + std::to_string(i)).c_str());
    if (plan.straight > 1e-8)
        motion.line(plan.straight, Element::Turn, (label + "-link").c_str());
    for (size_t i = 0; i < compiled.second.size(); ++i)
        motion.programme(compiled.second[i], Element::Turn,
                         (label + "-arrival-turn-" + std::to_string(i)).c_str());
    if (norm(motion.cursor.position - endPosition) > 1e-7 ||
        norm(motion.cursor.tangent - direction(endHeading)) > 1e-8)
        throw std::runtime_error(std::string(name) +
                                 ": emitted approach lost its endpoint contract");
}
} // namespace coaster
