# Force-profile scope and standards references

The game uses the higher historical F2291-23b force curves for an upright, individually contained rider with special uplift restraints and padded upper-torso restraints. This is an explicit design assumption for the train, not evidence that the current visual model supplies those protections. The runtime checks every front/middle/rear solver history at 960 Hz and again at 1920 Hz. Raw peak guards and the strongest ten-second exposure are separate quantities; neither substitutes for the signed duration and combined-load assessment.

ASTM currently lists [F2291-26](https://store.astm.org/f2291-26.html) as active (publisher page updated July 15, 2026). Its public scope separates restraint/clearance criteria, acceleration limits and structural requirements, and distinguishes mandatory annexes from nonmandatory appendices. The complete current acceleration clauses and figures have not been obtained for this project. [F2137-19(2025)](https://store.astm.org/f2137-19r25.html) is the publisher’s active measurement practice covering instrumentation, acquisition, documentation and standardized ride characterization. It does not provide the missing I305/Pantherian recordings.

The numerical source is [Rohde's March 2024 second edition](https://www.vdv-freizeittechnologie.de/files/bilder-vdv/pdf/Rohde-Accelerationlimits-2nd-edition-2024.pdf), figures 7.1, 7.2, 7.4 and 7.23, explicitly discussing F2291-23b. The higher X/Z branches were independently checked against the ASTM figures reproduced in [JP Research's 2019 presentation](https://pspe-philly.org/programs/PSPE_ASTM_%20presentation.pdf). Indexed [historical primary text](https://cdn.standards.iteh.ai/samples/116183/3ff5ecf855514b72a94193b5edd7b188/ASTM-F2291-23b.pdf) confirms filtering, paired ellipses, onset and sequencing clauses; that old PDF's direct URL later returned 404. Current-edition certification is not claimed.

## Selected higher force profile

The fixed profile and modelling requirements are owned by [force_envelope.hpp](include/coaster/force_envelope.hpp); [force_envelope.cpp](src/force_envelope.cpp) owns its curves and assessment. Coordinates below are `(duration seconds, force g)`, with linear interpolation. Force axes are the rider's vertical Z, lateral Y and forward X axes, including gravity.

| Direction | Curve vertices |
|---|---|
| +Z | (0.2,6), (1,6), (2,4), (4,4), (5,3), (11.8,3), (12,2) |
| -Z, special restraint | (0.2,-2.8), (1,-2.2), (3,-1.5), (4,-1.5), (7,-1.1) |
| +Y / -Y | (0.2,±3), (1,±3), (2,±2) |
| +X | (0.2,6), (1,6), (2,4), (4,4), (5,3), (11.8,3), (12,2.5) |
| -X, padded upper-torso exception | (0.2,-3.5), (2,-3.5), (3,-2.5), (4,-2.5), (5,-2) |

The enhanced negative-X branch is expressly available to an upright upper-torso restraint when it is appropriately padded, minimizes forward motion and has load-buildup onset below 15 g/s. It does not require changing the rider to a prone posture. The implementation checks `-dGx/dt` throughout an enhanced negative-load event, including the lead-in below 2 g. It does not mistake a negative signed slope for automatic compliance or impose an unsupported absolute release-rate condition. Extended negative Z needs special restraints justified by ride analysis; a generic lap bar or the OTS label alone does not establish this. Positive X requires maintained back/head support. See the [modelling requirements](../art/MODELLING_BRIEF.md).

All axes are filtered using a steady-state-initialized, fourth-order, single-pass 5 Hz Butterworth. Every magnitude level of each connected excursion is considered, rather than averaging a peak away or using only its sign duration. The 200 ms domain is explicit; separate raw guards keep shorter spikes inside the selected peak range. Positive Z at or above 2 g cannot persist beyond 40 seconds; sustained lateral loads have a 90-second extent.

Each paired-axis quadrant uses the applicable signed 200 ms radii, and an excursion outside its ellipse for at least 200 ms fails. There is no invented isotropic or three-axis sphere. Horizontal sustained-event reversals within 200 ms of their peaks receive the published half-limit check. Centered 100 ms least-squares onset is reported on all axes.

After a sustained nonpositive Z event, the 0-to-2 g rise must take at least 133 ms. Following three seconds of negative Z, the reduced positive curve is (0.2,5), (1.5,5), (2,4), (2.5,2), for six seconds after the transition back to positive Z; the normal curve then resumes. At exactly three seconds the implementation follows the figure's `>=3 s`, rather than the prose's `>3 s`. The project additionally prevents a positive interruption shorter than 200 ms from resetting accumulated negative exposure. Both choices are identified as project interpretations, not additional ASTM wording.

The existing `maxJerkGps` guard remains an instantaneous analytic vertical derivative. Configurable horizontal raw-rate guards use solver-step differences. These are separate from filtered 100 ms onset. Reports retain the distinction and the profile identifier; all assessed force-utilization and filtered-range/onset metrics participate in mandatory half-step convergence. Saves recompute the assessment from canonical geometry and physical operations. The independent terrain/train/support clearance certificates are unchanged by the force-profile choice.

## Whole-ride engineering scope

F2291 is a ride-design standard, not just a force envelope. The [publisher's current scope](https://store.astm.org/standards/f2291) also covers general design, patron restraint/containment and clearance, loads/strength, safety controls, electrical and mechanical systems, access/guarding, welding, fasteners and documentation. Relevant requirements must be assessed together; the public section list does not supply their detailed criteria.

The core currently checks a modeled train/reach sweep, terrain and explicit support clearance, support connectivity, finite-train motion and stopping. Those are useful game constraints. Support members are geometric families, not stress/buckling/fatigue-qualified structures. The envelope is not an anthropometric restraint qualification; station return is not an emergency-stop, rollback, evacuation or redundant-control assessment. These distinctions also apply to user-authored future layouts.

Technology plausibility needs a work and load budget in addition to a successful integration. The default train has six 1500 kg cars (9000 kg total). Reaching 50 m/s from rest requires at least 11.25 MJ and 8.04 MW average mechanical power in 1.4 s before drag, rolling losses or drive inefficiency. At 40 m/s² the total thrust is 360 kN; maintaining that acceleration at 80 m/s needs 28.8 MW mechanical power. These are model calculations, not measured drive specifications. Operations store per-car force/power limits; installed train-wide capacity cannot be read as one car's value.

[Intamin's Falcon's Flight report](https://www.intamin.com/2026/01/15/six-flags-qiddiya-city-falcons-flight/) establishes a real250 km/h LSM coaster with terrain-dependent launch placement. [InTraSys's German product description](https://intrasys-gmbh.com/lineare-antriebstechnik/) describes systems up to200 kN and ton-scale objects reaching50 m/s in seconds. It does not establish the game's simultaneous train mass, acceleration and high-speed power requirements. Higher combined thrust may be physically plausible with a purpose-designed distributed drive, but component ratings, electrical storage, cooling/duty cycle, wheel speed/loads, structure and failure cases remain unverified. Do not label authoring-derived capacities as certified or commercially demonstrated hardware, or raise capacities silently to satisfy a target.

Source FVD intent, finite-train simulation, authentic telemetry and engineered equipment feasibility are separate evidence. A source's high speed or force history must survive its actual joins, terrain, operations and all rider positions. A rejected combination remains rejected, and the authentic reference requirement remains independent.

## Diagnostic tool

Run `python native/tools/ride_load_profile.py TRACE.json --threshold vertical:gt:2 --threshold vertical:gt:3 --threshold=vertical:lt:-0.5 --threshold lateral:gt:0.5 --threshold=lateral:lt:-0.5 --output FRESH.json`. Add `--include-onset-series` for each assessed centre and signed slope; otherwise signed minima/maxima and their times are retained. Thresholds are explicit descriptive bins, never default safety limits. Existing output files are refused.

Input is the existing CLI trace: nominally 60 Hz presentation frames, with `seatOrder` front/middle/rear and signed `[vertical,lateral,longitudinal]` forces in g. These are unfiltered presentation samples, not the raw 960/1920 Hz solver series or measured rider data. The tool cannot recover peaks or impacts omitted by trace sampling. It does not use or modify geometry, speed, physics, saved rides or acceptance.

Threshold crossing times use linear interpolation only between observed frames. Positive-duration equality plateaus split strict exceedance runs; isolated equality touches do not. Runs touching the trace boundary are identified. Onset fits use actual timestamps only inside completely observed centred 100 ms windows; no filtering, resampling or edge extrapolation occurs. Invalid/nonfinite/non-monotone samples and nonuniform interior gaps are rejected; the CLI’s shorter final interval is supported. This raw-window diagnostic is not the filtered standard evaluation method.

This diagnostic remains useful for inspecting presentation traces. The actual acceptance assessment is the full-rate runtime implementation above. A future claim about F2291-26 needs that edition's complete clauses, applicable restraint/population analysis and the broader design assessment; this does not prevent engineering against the explicit verified historical profile now.

Focused validation: `python -m unittest native/tools/test_ride_load_profile.py -v` checks analytic constants, signed ramps, a quadratic centred fit, exact crossing durations, plateau boundaries, shortened terminal timing, missing-window rejection, malformed inputs and preservation of existing files. Synthetic tests are not reference evidence.
