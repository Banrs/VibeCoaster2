#pragma once

#include "coaster/coaster.hpp"

namespace coaster {

// Public callers retain the stale-cache verification performed by
// buildClearanceSweep. Recheck paths that have already rebuilt and verified
// the canonical track may use this exact sweep builder without rebuilding it.
ClearanceSweep buildClearanceSweepVerified(const Track&, const TrainConfig&, Cancel cancel = {});

// Standalone source sections may be open; this is not complete ride acceptance.
ValidationReport validateSelfClearance(const Track&,const TrainConfig&,double minClearance=0,Cancel cancel={});

double minimumSweptGroundClearance(const ClearanceSweep&, const Terrain&, Cancel cancel = {});

ValidationReport validateStation(const Track&, const Terrain&, const TrainConfig&, const StationGeometry&,
    const ClearanceSweep&, Cancel cancel = {});
ValidationReport validateDesignStructures(const Design&, const ClearanceSweep&, Cancel cancel = {});

ValidationReport validateGeometry(const Track&, const Terrain&, const Limits&, const TrainConfig&,
    const std::vector<Support>&, const ClearanceSweep&, Cancel cancel = {});

}
