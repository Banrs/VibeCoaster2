#pragma once
#include "coaster/coaster.hpp"
#include <optional>
namespace coaster {
// Called only after all native acceptance stages succeed. The snapshot stays
// in memory; neither a file checksum nor stored telemetry can create one.
void freezeAcceptedRevision(Design&);
// Derived Loop brake/link sources keep their own section labels and their saved parent.
std::optional<size_t> recipeOwnerForSection(const Design&,const RideSection&);
// Actual canonical element extents and the conservative embedded-foundation datum.
ValidationReport validateReferenceDimensions(const Design&,Cancel cancel={});
// Compare the actual ascent of each physical seat; refined tracks remap the phase endpoints.
ValidationReport validateInversionReferenceForces(const Design&,const SimulationResult&,const std::function<double(double)>& distanceMap={});
}
