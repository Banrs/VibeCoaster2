#pragma once
#include "coaster/simulation.hpp"

namespace coaster {
void saveDesign(const Design &, const std::filesystem::path &, const Cancel &cancel = {});
Design loadDesign(const std::filesystem::path &, const Cancel &cancel = {});
} // namespace coaster
