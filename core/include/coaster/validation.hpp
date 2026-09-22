#pragma once
#include "coaster/simulation.hpp"
#include "coaster/scene.hpp"

namespace coaster {
// Fresh source/dynamics and scene-geometry evidence for the explicit model.
// Physical motor coverage and release checks are separate; this is not whole-standard approval.
struct RideValidation {
    ReplayResult replay, refinedReplay;
    Clearance clearance;
    SceneGeometry scene;
    std::array<Simulation, 3> operating;
    // Temporal only, spatial only, then both, relative to the same nominal run.
    std::array<Simulation, 3> refined;
    double seconds{}, setupSeconds{};
    std::array<double, 9> checkSeconds{};
    unsigned workers{};
};
// Rebuilds the operating reference and all evidence. No stored result or
// checksum can skip a check. The caller's cancellation callback is serialized.
void validateRide(Design &, const Cancel &cancel = {});
} // namespace coaster