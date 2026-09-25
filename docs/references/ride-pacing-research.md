# Ride pacing and terrain reference for the 180-second redesign

Footage observations below remain source material. Proposed element counts, durations and shapes are historical; the latest user direction in [HANDOFF.md](../../HANDOFF.md) supersedes them.

24 September 2026. This is an authoring reference, not a survey of either ride. Timings below come from **video presentation time**, including editing and station footage. They are broad visual phase windows, not exact track ports. The Falcon's Flight telemetry overlay and the local RFDB export are different recordings of the same ride; do not synchronize their timestamps or treat displayed hundredths of a g as calibrated accuracy.

## What the primary references show

[Intamin's Falcon's Flight account](https://www.intamin.com/project/falcons-flight/) specifies an approximately **3 min 25 s / 205 s** ride, a 55 m twisted opening drop, gentle twists/curves/hills before the cliff, a 150 km/h launch toward the 195 m summit, an outward-banked summit turn **followed by more turns**, a holding brake immediately before the cliff dive, a tunnel and final launch to 250 km/h, a 165 m camelback, and elongated high-speed turns and hills on the return. [Intamin's company history](https://www.intamin.com/company/about-us/) gives 4,325 m of track. The designer explicitly describes gentle banking and drawn-out curves to control forces. The requested 180 s target is therefore shorter than Intamin's published ride duration; it should preserve the relative rhythm rather than claim to duplicate every second.

[Six Flags Over Texas](https://www.sixflags.com/overtexas/attractions/tormenta-rampaging-run) publishes Tormenta Rampaging Run's 309 ft height, 285 ft/95° first drop, **three-second edge hold**, 87 mph speed, 218 ft Immelmann and 179 ft loop. Its [official POV](https://www.youtube.com/watch?v=LesJ9BGNk8w) and the local frame sequence show that these landmarks are embedded in a longer layout: first Immelmann, loop, connector into a high block-brake run, another steep drop, a further inversion/turn sequence, then return braking. The exact labels of the later maneuvers are a visual/local-audit interpretation, not names supplied in the park's published attraction text. The local `docs/reference-force-audit.md` identifies the second Immelmann and cutback separately, with approximate RFDB-to-POV alignment uncertainty of ±2 s.

## Rewatched local POV phases

The saved [CoasterTalk consumer-telemetry POV](https://www.youtube.com/watch?v=0UaOSBGSx20) is `out/reference-videos/0UaOSBGSx20.mp4` (214 s edited clip). The local contact sheets are `out/reference-audit/falcons-phase/contact-48-72-4-1.jpg`, `contact-84-112-2-1.jpg`, `contact-84-112-2-2.jpg`, and `contact-150-180-5-1.jpg`. The exact readings and source caveats are retained in [falcons-flight-video-observations.json](falcons-flight-video-observations.json).

| Edited FF video time | Visible rhythm | Evidence and confidence |
| --- | --- | --- |
| ~48–72 s | Twisted opening drop gives way to several hills, valleys and banked connectors on approach to the cliff launch. | Direct frame observation; the sampled vertical overlay alternates −0.25/−0.10 g at 48/52 s, +2.29/+2.32 g at 56/60 s and light +0.48/+0.36 g at 64/68 s. Broad rhythm only. |
| ~72–84 s | Straight cliff-launch approach and rising track; arrival at the upper cliff turn. | Direct frame observation, approximate endpoints. |
| ~84–108 s | **Active clifftop travel:** outward cliff-edge bank, successive wide upper turns, low-load crests and loaded valleys while following the slope. | Direct two-second frame review: 84 s upper turn (+1.96 g); 88 s light crest (+0.55); 92 s loaded transition (+1.82); 96 s banked crest (+0.66); 98 s valley (+1.91); 102 s light crest (+0.24); 104–106 s final banked transition (+1.63/+1.20). There are several phase changes, but frames cannot establish an exact turn count. |
| ~108–115 s | Route straightens toward lip; braking begins at ~112 s and is followed by a short hold before commitment. | 108/110 s show near-1 g straight approach; 112 s overlay shows −0.93 longitudinal g during braking. Hold endpoint inferred from the later dive view, so do not hard-code this as a measured seven-second brake. |
| ~115–131 s | Cliff dive, tunnel, final launch and approach to the camelback. | Broad frame landmarks (120 s descending, 130 s tunnel/launch area). |
| ~133–149 s | Long camelback, including loaded ascent, negative crown, descending recovery. | Detailed visual/overlay phase record in local observation JSON; **about 15–16 s** of element body. |
| ~149–190+ s | Extended high-speed return, alternating long banks and hills, then park-side approach. | Direct sampled frames at 150/155/160/165/170/175/180/190 s. Return is a major act, not a short closure connector. |

The FF clifftop's **active** interval is about 24 s from 84 to 108 s, followed by a distinct brake/hold interval. This directly addresses the current mismatch where waiting outlasts winding. A 25–30 s authored moving clifftop and roughly 4–6 s lip brake/hold is a defensible project target, subject to geometry and ride-speed validation. Six long, readable bank/crest arcs over this interval are an original elaboration of the observed rhythm; Intamin does not document six discrete turns. Keep radii long and roll-rate transitions progressive, especially at cliff-launch speed.

The [official Tormenta POV](https://www.youtube.com/watch?v=LesJ9BGNk8w) is saved as `out/reference-videos/LesJ9BGNk8w.mp4`; inspect `out/reference-audit/tormenta-alignment/contact-77-99-1-{1,2}.jpg` and `contact-109-129-1-{1,2}.jpg`. Its 131 s clip has a long lift and only a roughly 50 s post-drop gravity run, so it is a **maneuver and cadence** reference rather than a 180 s duration template. The visible low-to-ground valleys around video 79–81 s and 93–95 s and the rising inversion approaches show that a record-height coaster can still spend consequential time near local grade.

## Authoring budget for a 180 s ride

These are **design targets**, not measured FF chapter durations. They sum to 180 s and intentionally keep the active clifftop longer than the lip wait. Treat ±5–10 s per broad act as acceptable when the total and flow are better.

| Act | Target seconds | What should fill the time |
| --- | ---: | --- |
| Departure, opening rise and twisted first descent | 28 | Establish scale, then commit to a real opening drop. |
| Pre-climb terrain act | 24 | Several discrete low hills and banked valleys, with visible terrain proximity; no anonymous flat coast. |
| Cliff launch and climb | 12 | A readable acceleration and long uphill approach. |
| Moving clifftop | 24 | Outward-bank reveal, alternating wide crests/loaded connectors and at least two additional changing-view turns. |
| Lip brake and hold | 6 | Short anticipation beat after the windy section. |
| Cliff dive, tunnel and final launch | 18 | Clear commitment, low tunnel/ground rush, renewed acceleration. |
| Camelback | 16 | Loaded ascent, sustained crest/airtime and recovery. |
| High-speed return, including inversion/ravine signature acts | 40 | Elongated banks and hills plus the project's original elements; avoid one long straight coast. |
| Final brakes and station approach | 12 | Deceleration and closure. |
| **Total** | **180** | |

Preserve coherent speed and force in each act: extending time by adding long, shaped travel is preferable to slowing the train excessively or using rapid roll/heading changes to manufacture force. The early hills and late high-speed turns/hills are the clearest FF elements currently worth restoring beyond the major records. Tormenta's block-brake-to-second-drop reset and distinct post-brake inversion/cutback sequence are useful structural cues if they fit the original layout. They need not be copied element-for-element.

## Ground-relative height

The FF contact sheets show the track near landscaping and roads during early hills and near the **sloped cliff surface** at ~100–106 s, despite being high above the lower plain. The official Tormenta POV likewise shows low valleys close to local grade. Neither source provides surveyed rail-to-ground distances, and camera height/vehicle geometry prevent a reliable numeric clearance estimate. Consequently, the reference supports reducing **time- and distance-weighted rail height above the local rendered terrain**, not guessing a universal height from an image or lowering the certified minimum blindly.

Use the existing [clearance model](../clearance-model.md): its swept rider/vehicle envelope and exact terrain triangles decide the non-negotiable local minimum. For visual authoring, reduce the ordinary rail datum and broad landform/track offset together where feasible, favor a substantial share of safe low passes in the opening, clifftop and return, and inspect front-seat sightlines. Keep giant supports and footing out of the repeated forward view where the structural solve allows it. The model's existing ~2 m ordinary rail intent and ~3.1 m winding-section mean are **project parameters**, not measured real-coaster dimensions; compare the newly generated mean/distribution with the previous candidate before claiming improvement.

## Reference limits

- The CoasterTalk overlay explicitly describes consumer sensors and edited presentation. Its force samples establish broad progression only. The local RFDB Watch recording has different timing, seat and run; see [reference-force-audit.md](../reference-force-audit.md).
- The official ride pages give credible dimensions and narrative order, but no complete surveyed geometry, exact element ports or ground-clearance distribution.
- FF's published 205 s duration and the requested 180 s project target should be reported distinctly. Tormenta's 131 s official video includes station and lift footage and does not imply a 131 s moving gravity ride.
