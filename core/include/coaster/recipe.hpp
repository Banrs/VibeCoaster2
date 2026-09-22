#pragma once
#include "coaster/track.hpp"
#include <filesystem>
#include <memory>

namespace coaster {
struct Recipe {
    int version{1};
    unsigned seed{42};
    std::string style{"balanced"};
    double plateau{210}, openingHeight{75}, camelbackHeight{224.65474672444046}, loopHeight{135},
        immelmannHeight{95};
    double topSpeedKph{300}, activeSeconds{180}, terminalSeconds{8};
};
Recipe readRecipe(const std::filesystem::path &);
void writeRecipe(const Recipe &, const std::filesystem::path &);
void validateRecipe(const Recipe &);
struct Simulation;
struct Design {
    Recipe recipe;
    Track track;
    std::vector<std::string> notes;
    std::shared_ptr<Simulation> baseline;
};
void validateAuthoring(const Design &, const Cancel &cancel = {});
Design generate(const Recipe &, const Cancel &cancel = {});
Program launch(const State &, double targetSpeed, double seconds);
} // namespace coaster
