#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// F2291-23b sections 7.1.4.8, 7.1.5-7.1.7 and Rohde, March 2024 (2nd
// ed.), figures 7.1, 7.2, 7.4, 7.15, 7.20-25. This fixed profile is not F2291-26
// qualification. The selected upright profile uses the special-restraint -Gz
// curve and the padded upper-torso exception to the ordinary -Gx limits.
inline constexpr const char* forceEnvelopeProfile="historical-f2291-23b-upright-extended-padded-ots-2024/project-1";
inline constexpr const char* forceEnvelopeModelRequirements=
    "Upright individually contained seating; appropriate backrest and headrest with maintained rider contact; "
    "padded upper-torso/over-shoulder restraint minimizing forward motion; extended negative-Gz special restraints "
    "justified in ride analysis; enhanced negative-Gx load buildup onset below 15 g/s. "
    "These are required modelling/design assumptions, not a claim that the current visual model proves compliance.";
enum class ForceAxis { Vertical, Lateral, Longitudinal };
double historicalForceLimit(ForceAxis,bool positive,double durationSeconds,bool afterUplift=false);

// All uniform solver samples, in seat-local g including gravity. A steady-state
// initialized, single-pass 5 Hz four-pole Butterworth precedes this assessment.
// Excursions inspect all magnitude levels, not averages or sign-onset duration.
// Raw peaks/impacts remain separate caller checks. Finding.distance is elapsed
// SECONDS here, so callers must map it before presenting a track distance.
//
// Reduced positive limits apply for 6 s AFTER transition to positive Gz, then
// normal limits resume (7.1.7.1). At exactly 3 s we follow Fig. 8's >=3 s rather
// than the prose's >3 s. Explicit conservative PROJECT anti-blip policy: a
// positive gap <200 ms does not reset accumulating negative exposure.
// Pairwise quadrant ellipses use signed 200 ms radii; outside excursions shorter
// than 200 ms are excluded (7.1.5.1). No unpublished three-axis equation is used.
// Vertical transitions use the explicit 0->2 g minimum 133 ms (7.1.7.2);
// centered onset is reported on all axes without inventing a blanket rate cap.
// The enhanced -Gx curve requires its actual <15 g/s buildup condition across
// the entire negative event, including the lead-in below the ordinary 2 g limit.
// This qualification also applies to events shorter than 200 ms.
// When this condition fails, combined/reversal limits retain the ordinary OTS -Gx radius;
// an event exceeding 2 g is rejected for using the unavailable higher exception.
ForceEnvelopeAssessment assessForceEnvelope(const std::array<std::vector<double>,3>& axes,
    double step,Cancel cancel={});
}
