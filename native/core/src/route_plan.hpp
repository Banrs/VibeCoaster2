#pragma once
#include "recipe_compiler.hpp"
#include "authoring.hpp"

namespace coaster {
// The route is an ordered plan, not a search candidate. Element IDs carry source,
// hardware and measured-section identity through compilation and replay.
struct RouteEnergy {
    double downhillSpeedCorrection{};
    std::unordered_map<std::string, double> portSquaredSpeedCorrection;
};
struct RoutePlacement {
    double climbHeading{}, signatureHeading{};
};
struct ShelfWaypoint {
    double headingDegrees, heightOffset, lengthMeters, pitchCurvature, yawCurvature;
};
struct RoutePlan {
    GenerationRequest request;
    double hand{};
    // Winding shelf: paired opposing turns, then a committed approach to the
    // dive. These are route intent, independent of energy-fit observations.
    std::array<ShelfWaypoint, 3> shelf{
        {{55, -18, 220, .003, 0}, {-35, -1, 240, -.003, -.004}, {0, -10, 300, 0, 0}}};
    explicit RoutePlan(const GenerationRequest &input) : request(input) {
        request.style.automaticTrims = false;
        if (request.recipe.elements.empty())
            request.recipe = defaultRideRecipe();
        if (request.style.returnStyle < 0)
            request.style.returnStyle = int(elementSeed(input.seed, "return-style") & 1);
        std::string error;
        if (!validateRecipe(request.recipe, error))
            throw std::invalid_argument(error);
        const auto cliff =
            std::find_if(request.recipe.elements.begin(), request.recipe.elements.end(),
                         [](const auto &e) { return e.role == RideRole::CliffApproach; });
        if (cliff != request.recipe.elements.end())
            shelf.back().headingDegrees =
                std::get<CliffParameters>(cliff->parameters).approachHeadingDegrees;
        hand = (elementSeed(input.seed, "landscape-hand") & 1) ? 1. : -1.;
    }
};
Design composeRoute(const GenerationRequest &, const RouteEnergy &, const RoutePlacement &,
                    std::vector<RecipePort> &, Cancel, RecipeCompileMode);
} // namespace coaster
