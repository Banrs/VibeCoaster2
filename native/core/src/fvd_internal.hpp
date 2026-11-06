#pragma once
#include "coaster/fvd.hpp"

namespace coaster {
// Integrates a source boundary for local authoring solves without building a
// canonical track or claiming assessment. Final sources use designFvdSection.
FvdSample integrateFvdEndpoint(const FvdRequest &, Cancel = {});
} // namespace coaster
