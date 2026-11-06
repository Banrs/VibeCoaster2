#pragma once
#include "coaster/coaster.hpp"
#include <unordered_map>
#include <stdexcept>
#include <optional>

namespace coaster {
inline constexpr double recipeApproachNormalG = 2.4, recipeApproachBankRampSeconds = 1.8;
enum class RecipeCompileMode { ClosedCircuit, EnergyCalibrationPrefix };
struct RecipePort {
    std::string id;
    double distance{}, speed{};
};
struct RecipeEnergyEstimate {
    std::unordered_map<std::string, double> energyCorrection;
};
struct RecipeCompileFailure : std::runtime_error {
    Design partial;
    std::vector<RecipePort> reachedPorts;
    std::optional<RecipePort> pendingPort;
    RecipeCompileFailure(std::string message, Design design, std::vector<RecipePort> reached = {},
                         std::optional<RecipePort> pending = {})
        : std::runtime_error(std::move(message)), partial(std::move(design)),
          reachedPorts(std::move(reached)), pendingPort(std::move(pending)) {}
};
struct RecipeBootstrapObservation {
    RecipePort port;
    double estimatedSpeed{};
    bool provisional{};
};
struct RecipeBootstrapResult {
    bool corrected{}, cancelled{};
    double maximumSpeedCorrection{}, continuationMeters{};
    std::vector<RecipeBootstrapObservation> observations;
    ValidationReport report;
};
// A temporary finite-train replay over a short analytic continuation may seed
// authoring feedback. It never changes the partial Design or produces a ride.
RecipeBootstrapResult bootstrapRecipeEnergy(const RecipeCompileFailure &, RecipeEnergyEstimate &,
                                            Cancel = {});
// Produces final geometry, source frames and hardware before acceptance reads it.
Design compileRoute(const GenerationRequest &, Cancel = {}, Progress = {});
double plannedDepartureSeconds(double acceleration, const TrainConfig &, const Limits &);
double departureRamp(double acceleration, const Limits &);
} // namespace coaster
