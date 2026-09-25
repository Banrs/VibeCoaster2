#pragma once
#include "coaster/coaster.hpp"
#include <unordered_map>
#include <stdexcept>
#include <optional>

namespace coaster {
inline constexpr double recipeLoopEntrySpeed=48;
inline constexpr double recipeClimbSetupHeading=0.233723406495529;
inline constexpr double recipeApproachNormalG=2.4,recipeApproachBankRampSeconds=1.8;
enum class RecipeCompileMode { ClosedCircuit, EnergyCalibrationPrefix };
struct RecipePort {std::string id;double distance{},speed{};};
struct RecipeFeedback {
    double peakSpeedCorrection{};
    double camelbackTailCutSeconds{};
    bool compactReturn{};
    double cliffHeadingCorrection{},cliffDepartureHeadingCorrection{};
    std::unordered_map<std::string,double> energyCorrection;
    std::unordered_map<std::string,double> brakeAccelerationCorrection;
};
struct RecipeCompileFailure : std::runtime_error {
    Design partial;
    std::vector<RecipePort> reachedPorts;
    std::optional<RecipePort> pendingPort;
    RecipeCompileFailure(std::string message,Design design,std::vector<RecipePort> reached={},std::optional<RecipePort> pending={})
        :std::runtime_error(std::move(message)),partial(std::move(design)),reachedPorts(std::move(reached)),pendingPort(std::move(pending)){}
};
struct RecipeBootstrapObservation {
    RecipePort port;double estimatedSpeed{};bool provisional{};
};
struct RecipeBootstrapResult {
    bool corrected{},cancelled{};
    double maximumSpeedCorrection{},continuationMeters{};
    std::vector<RecipeBootstrapObservation> observations;
    ValidationReport report;
};
// A temporary finite-train replay over a short analytic continuation may seed
// authoring feedback. It never changes the partial Design or produces a ride.
RecipeBootstrapResult bootstrapRecipeEnergy(const RecipeCompileFailure&,RecipeFeedback&,Cancel={});
// Compiles ordered typed intent into one continuous authored circuit. These
// diagnostics are planning observations; independent finite-train replay rules.
Design compileRecipe(const GenerationRequest&,int attempt,const RecipeFeedback&,
                     std::vector<RecipePort>&,Cancel,RecipeCompileMode mode=RecipeCompileMode::ClosedCircuit);
double plannedDepartureSeconds(double acceleration,const TrainConfig&,const Limits&);
double departureRamp(double acceleration,const Limits&);
}
