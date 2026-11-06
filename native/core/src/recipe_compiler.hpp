#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>

namespace coaster {
struct RecipePort {
    std::string id;
    double distance{}, speed{};
};
struct RecipeCompileFailure : std::runtime_error {
    Design partial;
    std::vector<RecipePort> reachedPorts;
    RecipeCompileFailure(std::string message, Design design, std::vector<RecipePort> reached = {})
        : std::runtime_error(std::move(message)), partial(std::move(design)),
          reachedPorts(std::move(reached)) {}
};
// One forward draft. Energy advances over new geometry; authored sections are
// never rebuilt to repair an acceptance result.
Design compileRoute(const GenerationRequest &, Cancel = {}, Progress = {});
double plannedDepartureSeconds(double, const TrainConfig &, const Limits &);
double departureRamp(double, const Limits &);
} // namespace coaster
