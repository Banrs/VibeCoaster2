#pragma once
#include "coaster/coaster.hpp"
namespace coaster {
// Called only after all native acceptance stages succeed. The snapshot stays
// in memory; neither a file checksum nor stored telemetry can create one.
void freezeAcceptedRevision(Design&);
}
