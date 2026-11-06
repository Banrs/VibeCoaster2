#pragma once
#include "recipe_compiler.hpp"
#include "authoring.hpp"

namespace coaster {
// The route is an ordered plan, not a search candidate. Element IDs carry source,
// hardware and measured-section identity through compilation and replay.
struct RoutePlan {
    GenerationRequest request;
    double hand{};
    explicit RoutePlan(const GenerationRequest &input) : request(input) {
        request.style.automaticTrims = false;
        if (request.recipe.elements.empty()) {
            request.recipe = defaultRideRecipe();
            // Resolve seed variation into editable intent once. Element identity,
            // rather than recipe position, owns each independent random stream.
            auto riseFor = [&](const RecipeElement &element, double low, double high) {
                const auto bits = elementSeed(input.seed, element.id + "/rise");
                return std::lerp(low, high, double(bits >> 11) * 0x1.0p-53);
            };
            for (auto &element : request.recipe.elements) {
                if (element.role == RideRole::Opening)
                    std::get<HillParameters>(element.parameters).riseMeters =
                        riseFor(element, 28, 40);
                if (element.role == RideRole::Camelback)
                    std::get<HillParameters>(element.parameters).riseMeters =
                        riseFor(element, 95, 115);
                if (element.role == RideRole::CliffApproach)
                    std::get<CliffParameters>(element.parameters).summitHeightMeters =
                        request.targets.height;
                if (element.role == RideRole::DownhillLaunch)
                    std::get<OperationParameters>(element.parameters).targetSpeedKmh =
                        request.targets.speed * 3.6;
                if (element.role == RideRole::Wave)
                    std::get<TurnParameters>(element.parameters).riseMeters =
                        request.targets.inversionHeight;
                if (element.role == RideRole::Signature)
                    std::get<SweepParameters>(element.parameters).rollDegrees =
                        request.style.signatureRollDegrees;
            }
        }
        if (request.style.returnStyle < 0)
            request.style.returnStyle = int(elementSeed(input.seed, "return-style") & 1);
        std::string error;
        if (!validateRecipe(request.recipe, error))
            throw std::invalid_argument(error);
        hand = (elementSeed(input.seed, "landscape-hand") & 1) ? 1. : -1.;
    }
};
Design composeRoute(const GenerationRequest &, std::vector<RecipePort> &, Cancel, Progress = {});
} // namespace coaster
