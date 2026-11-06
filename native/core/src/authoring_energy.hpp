#pragma once
#include "coaster/coaster.hpp"

namespace coaster {

// Incremental planning motion. This carries physical momentum and each car's
// motor-entry epoch across source boundaries; it never certifies rider forces.
// A callback returns the unit track tangent at an absolute route distance.
class AuthoringEnergy {
  public:
    using Tangent = std::function<Vec3(double)>;

    explicit AuthoringEnergy(const TrainConfig &, double step = 1. / 240);
    double distance() const {
        return distance_;
    }
    double speed() const {
        return speed_;
    }
    double time() const {
        return time_;
    }

    // The sampled route must cover the whole train and one metre of RK stage
    // lookahead. Operations are immutable once observed and may be appended
    // only before the front car can reach them. Terminal station/trim replay is
    // deliberately outside the source planner's domain.
    void advanceTo(double centerDistance, const Tangent &, const std::vector<Operation> &,
                   Cancel = {});
    // Forecast with a short caller-owned continuation without committing that
    // provisional geometry or motion to the retained state.
    double forecastTo(double centerDistance, const Tangent &, const std::vector<Operation> &,
                      Cancel = {}) const;

  private:
    TrainConfig train_;
    double step_{}, halfTrain_{}, distance_{}, speed_{}, time_{}, stalled_{};
    std::vector<Operation> operations_;
    std::vector<std::array<double, 16>> entered_;

    void observeOperations(const std::vector<Operation> &);
    void enterOperations();
    double acceleration(double distance, double speed, double time, const Tangent &) const;
    std::pair<double, double> integrate(double step, const Tangent &) const;
};
} // namespace coaster
