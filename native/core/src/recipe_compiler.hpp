#pragma once
#include "coaster/coaster.hpp"
#include <unordered_map>
#include <stdexcept>

namespace coaster {
struct RecipePort {std::string id;double distance{},speed{};};
struct RecipeFeedback {
    double peakSpeedCorrection{};
    std::unordered_map<std::string,double> energyCorrection;
};
struct RecipeCompileFailure : std::runtime_error {
    Design partial;
    RecipeCompileFailure(std::string message,Design design):std::runtime_error(std::move(message)),partial(std::move(design)){}
};
// Compiles ordered typed intent into one continuous authored circuit. These
// diagnostics are planning observations; independent finite-train replay rules.
Design compileRecipe(const GenerationRequest&,int attempt,const RecipeFeedback&,
                     std::vector<RecipePort>&,Cancel);
double plannedDepartureSeconds(double acceleration,const TrainConfig&,const Limits&);
double departureRamp(double acceleration,const Limits&);
}
