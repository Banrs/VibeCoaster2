#include "coaster/recipe.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
int checks = 0;

void check(bool condition, const char* message) {
    ++checks;
    if (!condition) { std::cerr << "FAIL " << message << '\n'; std::exit(1); }
}

std::string replaceOnce(std::string text, const std::string& from, const std::string& to) {
    const auto original = text;
    const auto at = text.find(from);
    check(at != std::string::npos, "malformed recipe fixture mutation token exists");
    text.replace(at, from.size(), to);
    check(text != original, "malformed recipe fixture mutation changes payload");
    return text;
}

} // namespace

int main() {
    using namespace coaster;
    const auto recipe = defaultRideRecipe();
    std::string error;
    check(validateRecipe(recipe, error), "default recipe validates");
    check(std::count_if(recipe.elements.begin(), recipe.elements.end(), [](const auto& item) { return item.role == RideRole::Return; }) >= 2, "default recipe contains multiple ordinary returns");
    check(recipe.elements.front().role == RideRole::Station && recipe.elements[1].role == RideRole::Departure && recipe.elements.back().role == RideRole::Brakes, "default starts station/departure and ends brakes");
    check(recipe.elements[5].role == RideRole::CliffDrop && recipe.elements[6].role == RideRole::DownhillLaunch && recipe.elements[7].role == RideRole::Camelback, "cliff/downhill/camelback adjacency is explicit");
    check(std::get<OperationParameters>(recipe.elements[6].parameters).targetSpeedKmh == 0, "downhill LSM zero target derives the global speed");
    const auto& camelback = std::get<CamelbackParameters>(recipe.elements[7].parameters);
    check(camelback.profileScale > 0 && camelback.tailCutSeconds >= 0 && camelback.releaseSeconds > 0,
        "camelback uses the protected approved parameter family");
    check(camelback.minimumExitPitchDegrees > 0 && camelback.exitNormalG > 0,
        "camelback retains its approved exit constraints");

    check(std::string(roleName(RideRole::DownhillLaunch)) == "downhill-lsm", "role names are stable human tokens");
    check(std::string(anchorName(TerrainAnchor::ImmelmannShoulder)) == "immelmann-shoulder", "anchor names are stable human tokens");

    const auto seedA = elementSeed(42, "camelback");
    const auto seedB = elementSeed(42, "camelback");
    check(seedA == seedB && seedA == 15497263119282846436ull, "camelback seed has a fixed stable vector");
    check(elementSeed(42, "wave") == 11811803777310210526ull && elementSeed(43, "camelback") == 9650049089560291631ull, "seed changes with stable key or ride seed");

    const auto payload = recipePayload(recipe);
    check(!payload.empty() && payload.rfind("VIBECOASTER_RECIPE 1\n", 0) == 0, "canonical recipe payload has versioned header");
    RideRecipe parsed;
    check(parseRecipe(payload, parsed, error) && parsed == recipe, "canonical recipe round trips exactly");
    check(recipePayload(parsed) == payload, "canonical writer is byte stable");

    RideRecipe customized = recipe;
    std::swap(customized.elements[12], customized.elements[13]);
    customized.elements.insert(customized.elements.end() - 2,
        RecipeElement{"return-extra", RideRole::Return, TerrainAnchor::Ravine, SweepParameters{180, 8, 20, 0, 0, 5}});
    check(validateRecipe(customized, error), "compatible return reorder and unique insertion validate");
    RideRecipe customizedParsed;
    check(parseRecipe(recipePayload(customized), customizedParsed, error) && customizedParsed == customized,
        "compatible return reorder and unique insertion survive serialization");
    check(elementSeed(42, "camelback") == seedA, "existing element seed is unaffected by recipe edits");

    RideRecipe camelbackEdit = recipe;
    auto& editedCamelback = std::get<CamelbackParameters>(camelbackEdit.elements[7].parameters);
    editedCamelback.profileScale = 1.05;
    editedCamelback.tailCutSeconds = .8;
    check(validateRecipe(camelbackEdit, error), "approved camelback edits remain within its protected family");
    RideRecipe camelbackParsed;
    check(parseRecipe(recipePayload(camelbackEdit), camelbackParsed, error) && camelbackParsed == camelbackEdit,
        "approved camelback parameter edits round trip");

    RideRecipe hillReturn = recipe;
    hillReturn.elements[12].parameters = HillParameters{};
    check(validateRecipe(hillReturn, error), "return role accepts hill variant");
    hillReturn.elements[12].parameters = TurnParameters{};
    check(validateRecipe(hillReturn, error), "return role accepts turn variant");

    const auto malformed = [&](const std::string& text, const char* message) {
        RideRecipe retained = recipe;
        check(!parseRecipe(text, retained, error) && retained == recipe, message);
    };
    malformed(replaceOnce(payload, "lengthMeters=20", "lengthMeters=20 lengthMeters=21"), "duplicate fields reject without mutating output");
    malformed(replaceOnce(payload, "lengthMeters=20", "mystery=1 lengthMeters=20"), "unknown fields reject");
    malformed(replaceOnce(payload, "targetSpeedKmh=0", "targetSpeedKmh=nan"), "nonfinite numbers reject");
    malformed(replaceOnce(payload, "\"wave\" wave", "\"station\" wave"), "duplicate ids reject");
    malformed(payload + "trailing-junk\n", "trailing data rejects");
    malformed(replaceOnce(payload, "16\n", "65\n"), "element count bound rejects");
    malformed(replaceOnce(payload, "approved-camelback-v1 profileScale=", "approved-camelback-v1 twistDegrees=1 profileScale="),
        "protected camelback rejects hill-only twist fields");

    const auto folder = std::filesystem::temp_directory_path() / ("vibecoaster-recipe-tests-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    check(std::filesystem::create_directory(folder), "unique recipe test directory created");
    const auto path = folder / "roundtrip.recipe";
    check(saveRecipe(recipe, path.string(), error), "recipe saves atomically");
    RideRecipe loaded;
    check(loadRecipe(path.string(), loaded, error) && loaded == recipe, "recipe file loads exactly");
    auto edited = loaded;
    std::get<SweepParameters>(edited.elements[11].parameters).rollDegrees = 52;
    check(saveRecipe(edited, path.string(), error), "edited recipe overwrites atomically");
    RideRecipe editedLoaded;
    check(loadRecipe(path.string(), editedLoaded, error) && editedLoaded == edited, "edited recipe survives save/load/re-edit roundtrip");
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    std::filesystem::remove(folder, ignored);

    std::cout << "PASS " << checks << " recipe format, validation, keyed seeds and atomic roundtrip checks\n";
    return 0;
}
