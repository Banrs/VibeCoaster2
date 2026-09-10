# Specifications (as-published, bounded)

Scope: `source_register.json` is canonical. Published specifications below retain their 2026-09-06 retrieval date. The supplied-recording observations were computed on 2026-09-10. Marketing record claims, measured consumer recordings and configured game goals are distinct evidence.

## Verified

- Falcon's Flight (Intamin project page): height 195 m; cliff 195 m; camelback 165 m; 55 m twisted drop; top speed 250 km/h; 40 km/h lift, 150 km/h cliff-top launch; ~3:25 ride time; opened 31 Dec 2025. Marketed as tallest/fastest/longest; treat as claim. Length number and prior 163 m structure split were not observed on this fetch; see leads.
- Do-Dodonpa (RCDB 1423): source-native 160.8 ft height / 4081.3 ft length / 111.8 mph / 0-111.8 mph in 1.6 s / 160.8 ft loop. History text on same page: 49 m loop (39.7 m dia), 0-180 km/h in 1.56 s (was 0-172 km/h in 1.8 s), 1189 m to 1244 m, prior 52 m height and listed 4.3 g. Keep imperial and metric labels separate. 4.3 g is a prior-config press figure, not 10-s exposure. Closed 2 Oct 2016, reopened 15 Jul 2017 as Do-Dodonpa.
- Tormenta Rampaging Run (Six Flags Over Texas operator page): source-native 309 ft height / 285 ft drop at 95 deg / 87 mph / 4199 ft; Immelmann 218 ft; loop 179 ft. Marketed as six dive-coaster records; treat as claims. Keep imperial canonical; do not collapse height/drop/elements.

## Game goals (not world records)

Configured thresholds from `native/core/NUMERICS.md`: 220 m height, 75 m/s (270 km/h), 80 m inversion height, 0-50 m/s in 1.4 s. `physics-proof` scope only. Not independently confirmed records.

## Limits and leads

- Raw Pantherian 6839, Falcon's Flight 4804 and Tormenta 6383 files are supplied under `RFDB Data/`; see the observations below. The historical `processed/benchmark.json` predates them. A qualified multi-record reference remains unestablished; this does not prevent using their measured force histories for engineering.
- Wikipedia / thrillzing / PR / B&M blog / Fujikyu filing / Yomiuri / Asahi / method guides preserved in `source_register.json:leads_only` as unverified leads, not primary evidence.
- Prior 163 m / 4250 m Falcon numbers retained only as unverified leads.

## Supplied force observations

One local recording per ride. The existing source calibration-row projection and RFDB 11-sample display mean at nominal 50 Hz reproduce stored extrema; gravity stays included. Durations and S10 use piecewise-linear interpolation of observed samples. Raw files, hashes, source dates, signed lateral/longitudinal metrics and processing sensitivity are retained in the [analysis](../artifacts/flow-intent-v083-20260910/references/FINDINGS.md) and source register. Device stability, actual timestamp jitter and calibrated uncertainty remain unverified.

| Recording / observed seat | Selected span | Display vertical min–max | Total / longest >3g | Total <0g | Raw / display S10 |
|---|---:|---:|---:|---:|---:|
| [FF 4804](https://rideforcesdb.com/?id=4804), row 7 left | 158.02s | −0.986–3.894g | 4.476 / 1.103s | 14.274s | 20.251 / 20.100g·s |
| [Pantherian 6839](https://rideforcesdb.com/?id=6839), seat unspecified | 55.02s | −0.865–4.403g | 15.317 / 4.474s | 7.999s | 30.020 / 29.935g·s |
| [Tormenta 6383](https://rideforcesdb.com/?id=6383), row 2 seat 8 train 1 | 82.48s | −0.518–4.344g | 12.885 / 2.787s | 2.789s | 29.541 / 29.499g·s |

S10 is positive vertical exposure, not a peak or a safety limit. Pantherian's strongest one-second vertical mean is 4.160g, versus FF's 3.427g; its strongest ten-second window crosses several successive positive lobes. These observations support sustained, connected load sequences. FF and Pantherian each have roughly 2–2.5s total below−0.5g, divided into short intervals; they do not justify sustained maximum negative load everywhere. A +10% comparison to the one Pantherian observation is **33.022g·s raw / 32.928g·s displayed**, with between-recording spread unavailable. This is an honest single-observation comparison, not strict median qualification.

Terminal longitudinal histories also provide guidance: Pantherian stays below −0.5 g for 2.709 s, with a strongest one-second mean of −1.105 g; FF's late corresponding interval lasts 2.009 s. Such signed rider loads are not simply `dv/dt` on a grade and do not identify the brake technology. Motor/brake visual cues must follow actual applied operation force and physical hardware, not gravity-driven speed change.

## Standard applicability

The [ASTM catalog](https://store.astm.org/standards/f2291) identifies F2291-26. Its public scope separates restraint/clearance/containment, acceleration, structures, controls and other requirements; it does not publish a complete numerical acceptance profile. The inspected project contains archived public metadata with empty criteria, not a full standard. The [applicability review](../artifacts/flow-intent-v083-20260910/references/F2291_APPLICABILITY.md) maps current game checks and concrete gaps across those categories, with edition-specific public technical sources. The game's scalar force envelope is provisional and must not be relabeled as an ASTM limit.
