#include "authoring_energy.hpp"
#include <stdexcept>

namespace coaster {
namespace {
bool sameOperation(const Operation &a, const Operation &b) {
    return a.start == b.start && a.end == b.end && a.kind == b.kind &&
           a.targetSpeed == b.targetSpeed && a.maxForce == b.maxForce && a.maxPower == b.maxPower &&
           a.rampSeconds == b.rampSeconds && a.stopDeceleration == b.stopDeceleration &&
           a.stopOffset == b.stopOffset && a.exitFadeMeters == b.exitFadeMeters &&
           a.trimPeakSpeed == b.trimPeakSpeed && a.trimSensorLead == b.trimSensorLead;
}
// Identical bounded actuator laws to the independent full-train replay. The
// planner omits seat forces, history assessment, terminal capture and traces.
double positiveDemand(double value, double cap) {
    if (value <= 0)
        return 0;
    const double u = value / (cap * .05);
    if (u >= 1)
        return value;
    return value * std::pow(u, 4) * (35 + u * (-84 + u * (70 - 20 * u)));
}
double saturatedForce(double value, double cap) {
    if (value <= cap * .8)
        return value;
    if (value >= cap * 1.2)
        return cap;
    const double u = (value - cap * .8) / (cap * .4);
    return cap * .8 + cap * .4 *
                          (u - 7 * std::pow(u, 5) + 14 * std::pow(u, 6) - 10 * std::pow(u, 7) +
                           2.5 * std::pow(u, 8));
}
} // namespace

AuthoringEnergy::AuthoringEnergy(const TrainConfig &train, double step)
    : train_(train), step_(step) {
    if (train.cars < 1 || train.cars > 16 || !std::isfinite(train.spacing) || train.spacing <= 0 ||
        !std::isfinite(train.carMass) || train.carMass <= 0 ||
        !std::isfinite(train.rollingResistance) || train.rollingResistance < 0 ||
        !std::isfinite(train.dragCdA) || train.dragCdA < 0 || !std::isfinite(train.airDensity) ||
        train.airDensity < 0 || !std::isfinite(step) || step < 1. / 4000 || step > 1. / 120)
        throw std::invalid_argument("Invalid incremental authoring energy configuration");
    halfTrain_ = (train.cars - 1) * train.spacing * .5;
    distance_ = halfTrain_ + 30;
}

void AuthoringEnergy::observeOperations(const std::vector<Operation> &operations) {
    if (operations.size() < operations_.size())
        throw std::invalid_argument("Authored drives cannot be removed behind the energy cursor");
    for (size_t i = 0; i < operations_.size(); ++i)
        if (!sameOperation(operations[i], operations_[i]))
            throw std::invalid_argument("Authored drives cannot change behind the energy cursor");
    for (size_t i = operations_.size(); i < operations.size(); ++i) {
        const auto &op = operations[i];
        if (op.kind != DriveKind::Launch && op.kind != DriveKind::Boost &&
            op.kind != DriveKind::Brake)
            throw std::invalid_argument(
                "Source energy planning requires launch, boost or brake corridors");
        for (double value : {op.start, op.end, op.targetSpeed, op.maxForce, op.maxPower,
                             op.rampSeconds, op.exitFadeMeters})
            if (!std::isfinite(value))
                throw std::invalid_argument("Nonfinite authored drive");
        if (op.start < 0 || op.end <= op.start || op.targetSpeed < 0 || op.maxForce < 0 ||
            op.maxPower < 0 || op.rampSeconds < 0 || op.exitFadeMeters <= 0)
            throw std::invalid_argument("Invalid authored drive corridor");
        if (time_ > 0 && op.start < distance_ + halfTrain_ - 1e-7)
            throw std::invalid_argument("A new authored drive starts behind the front car");
        operations_.push_back(op);
        std::array<double, 16> entries;
        entries.fill(-1);
        entered_.push_back(entries);
    }
}

void AuthoringEnergy::enterOperations() {
    for (size_t j = 0; j < operations_.size(); ++j) {
        const auto &op = operations_[j];
        for (int car = 0; car < train_.cars; ++car) {
            const double at = distance_ + halfTrain_ - car * train_.spacing;
            auto &entry = entered_[j][car];
            if (at >= op.start && at < op.end) {
                if (entry < 0)
                    entry = time_;
            } else
                entry = -1;
        }
    }
}

double AuthoringEnergy::acceleration(double distance, double speed, double time,
                                     const Tangent &sample) const {
    double total = 0;
    for (int car = 0; car < train_.cars; ++car) {
        const double at = distance + halfTrain_ - car * train_.spacing;
        const Vec3 tangent = sample(at);
        if (!finite(tangent) || std::abs(tangent.z) > 1 + 1e-8)
            throw std::runtime_error("Source energy received an invalid track tangent");
        double force = -train_.carMass * gravity * tangent.z;
        for (size_t j = 0; j < operations_.size(); ++j) {
            const auto &op = operations_[j];
            if (at < op.start || at >= op.end)
                continue;
            const double entry = entered_[j][car];
            const double ramp =
                smooth((time - (entry < 0 ? time : entry)) / std::max(.001, op.rampSeconds)) *
                smooth((op.end - at) / op.exitFadeMeters);
            const double cap = std::min(op.maxForce, op.maxPower / std::max(1., speed));
            const double demand = (op.targetSpeed - speed) * train_.carMass * 4;
            if (op.kind == DriveKind::Brake)
                force -= saturatedForce(positiveDemand(-demand, cap), cap) * ramp;
            else
                force += saturatedForce(positiveDemand(demand, cap), cap) * ramp;
        }
        total += force;
    }
    total -= .5 * train_.airDensity * train_.dragCdA * speed * std::abs(speed);
    const double mass = train_.carMass * train_.cars,
                 rolling = mass * gravity * train_.rollingResistance;
    if (speed > 0 || total > rolling)
        total -= rolling;
    else if (total < -rolling)
        total += rolling;
    else
        total = 0;
    return total / mass;
}

std::pair<double, double> AuthoringEnergy::integrate(double dt, const Tangent &tangent) const {
    const double a = acceleration(distance_, speed_, time_, tangent);
    const double predictedMid = speed_ + a * dt * .5, vm = std::max(0., predictedMid),
                 sm = distance_ + speed_ * dt * .5;
    const double a2 = acceleration(sm, vm, time_ + dt * .5, tangent);
    const double v3 = std::max(0., speed_ + a2 * dt * .5), s3 = distance_ + vm * dt * .5;
    const double a3 = acceleration(s3, v3, time_ + dt * .5, tangent);
    const double v4 = std::max(0., speed_ + a3 * dt), s4 = distance_ + v3 * dt;
    const double a4 = acceleration(s4, v4, time_ + dt, tangent), am = (a2 + a3) * .5;
    double nextSpeed = std::max(0., speed_ + dt * (a + 2 * a2 + 2 * a3 + a4) / 6);
    double nextDistance = distance_ + dt * (speed_ + 2 * vm + 2 * v3 + v4) / 6;
    double stopping = 0;
    if (speed_ > 0 && a < 0 && predictedMid <= 0)
        stopping = a;
    else if (speed_ > 0 && am < 0 && speed_ + am * dt <= 0)
        stopping = am;
    if (stopping < 0) {
        const double stopTime = std::clamp(speed_ / (-stopping), 0., dt);
        nextDistance = distance_ + speed_ * stopTime + .5 * stopping * stopTime * stopTime;
        nextSpeed = 0;
        const double remaining = dt - stopTime;
        const double restart =
            acceleration(nextDistance, 0, time_ + stopTime + remaining * .5, tangent);
        if (restart > 0) {
            nextSpeed = restart * remaining;
            nextDistance += .5 * restart * remaining * remaining;
        }
    }
    return {nextDistance, nextSpeed};
}

void AuthoringEnergy::advanceTo(double target, const Tangent &tangent,
                                const std::vector<Operation> &operations, Cancel cancel) {
    if (!std::isfinite(target) || target < distance_ - 1e-8 || !tangent)
        throw std::invalid_argument(
            "Incremental source energy cannot run backwards or sample an absent route");
    observeOperations(operations);
    size_t steps = 0;
    while (distance_ < target - 1e-9) {
        if ((steps++ & 31) == 0 && cancel && cancel())
            throw std::runtime_error("CANCELLED");
        if (time_ >= 600)
            throw std::runtime_error("Source energy exceeded the supported ride duration");
        enterOperations();
        double dt = step_;
        auto next = integrate(dt, tangent);
        if (next.first > target) {
            // Locate the source boundary in time, preserving momentum instead
            // of snapping a full-step position or assigning a target speed.
            double low = 0, high = dt;
            // Resolve below the spatial stopping tolerance even at the
            // supported maximum speed. A coarser time search can round its
            // lower bound to zero and repeat this boundary forever.
            for (int i = 0; i < 40; ++i) {
                dt = (low + high) * .5;
                next = integrate(dt, tangent);
                if (next.first > target)
                    high = dt;
                else
                    low = dt;
            }
            dt = low;
            next = integrate(dt, tangent);
            if (target - next.first < 1e-8)
                next.first = target;
        }
        if (!std::isfinite(next.first) || !std::isfinite(next.second) || next.second > 200 ||
            next.first < distance_ - 1e-8)
            throw std::runtime_error("Incremental source energy left its physical domain");
        if (dt <= 0 || time_ + dt == time_)
            throw std::runtime_error(
                "Incremental source energy could not resolve its next boundary");
        distance_ = next.first;
        speed_ = next.second;
        time_ += dt;
        stalled_ = speed_ < .02 ? stalled_ + dt : 0;
        if (stalled_ > 3 && time_ > 3)
            throw std::runtime_error("Source energy stalled before the next authored boundary");
    }
}

double AuthoringEnergy::forecastTo(double target, const Tangent &tangent,
                                   const std::vector<Operation> &operations, Cancel cancel) const {
    auto forecast = *this;
    forecast.advanceTo(target, tangent, operations, std::move(cancel));
    return forecast.speed();
}
} // namespace coaster
