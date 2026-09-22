#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace coaster {

// Seat-fixed specific acceleration, including gravity, in g. The caller owns
// these uniformly spaced samples. Axis order is X (longitudinal), Y (lateral),
// Z (vertical); positive Z is eyes down. No display-rate resampling is allowed.
struct AccelerationSeries {
    std::span<const double> longitudinal;
    std::span<const double> lateral;
    std::span<const double> vertical;
    double stepSeconds{};
    double startTimeSeconds{};
    std::size_t seatIndex{};
};

struct AccelerationDiagnostic {
    std::string clause;
    std::string rule;
    std::string axis; // X, Y, Z, XY, XZ, YZ, or input
    std::string sign; // +, -, signed quadrant, or empty for input errors
    std::size_t seatIndex{};
    double startTimeSeconds{};
    double endTimeSeconds{};
    double actual{};
    double limit{};
    double utilization{}; // >1 fails; minimum-duration rules use limit/actual
};

struct AccelerationExtrema {
    double minimumG{};
    double maximumG{};
    double minimumTimeSeconds{};
    double maximumTimeSeconds{};
};

struct AccelerationOnset {
    bool available{};
    double minimumGps{};
    double maximumGps{};
    double minimumTimeSeconds{};
    double maximumTimeSeconds{};
};

struct AccelerationAssessment {
    bool performed{};
    bool passed{};
    bool cancelled{};
    std::vector<AccelerationDiagnostic> diagnostics;
    std::array<AccelerationExtrema, 3> filteredExtrema{}; // X, Y, Z
    std::array<AccelerationOnset, 3> onset100ms{};        // X, Y, Z; no project cap here
};

// Scoped F2291-25 numerical assessment for the upright base case, Class 4/5
// restraint with lower-body containment, backrest and headrest. This is not a
// whole-standard certification. See docs/acceleration-standard.md for scope,
// provenance, numerical conventions and unresolved impact-analysis cases.
AccelerationAssessment assessAccelerationF2291_25(const AccelerationSeries &series,
                                                  const std::function<bool()> &cancel = {});

// Independent boundary fixtures and already processed measurement data only.
// Input must already have undergone the edition-25 four-pole, single-pass,
// 5 Hz Butterworth processing. This entry point does not filter a second time.
AccelerationAssessment assessProcessedAccelerationF2291_25(const AccelerationSeries &processedSeries,
                                                           const std::function<bool()> &cancel = {});

} // namespace coaster
