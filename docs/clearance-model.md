# Clearance model used by the prototype

Clearance is a reach/containment problem, not a universal rail-height rule. The default optional gap outside the computed envelope is zero. The obsolete extra 1.6 m corner gate and minimum-height restrictions on ordinary piers have been removed. Numerical sweep bounds remain explicit and are computed from the actual envelope radius.

F2291-26 is the active edition in ASTM's catalogue. Its complete provisions were not publicly accessible during this revision. The accessible F2291-25 text specifies published 95th-percentile anthropometry, extended limb reach, containment and vehicle motion in 6.6.3; its 3.1.13 separately defines clearance beyond reach. This is a documented implementation of that guideline, not a certification against the unavailable current full standard.

The numeric example uses the NASA-hosted *Human Factors Design Guidelines*, PDF page 139 (printed page 142), reproducing MIL-STD-1472C figure 11: male 95th-percentile extended functional reach 1.012 m and seated overhead reach 1.469 m. The scanned table was rendered and checked because OCR misread the latter value. Limb extension and clearance are each 3 inches (0.0762 m), corresponding to the separate clauses above.

Prototype seating assumptions: two seat centres at +/-0.43 m; the seat pan is 0.45 m below the configured force-sample height; the torso is restrained and the legs remain within the closed footwell. These are declared model assumptions, not measured Intamin vehicle dimensions. They produce a half-width of 1.5944 m and default upper reach/clearance of 2.3714 m above the rail datum. The full 2.55 m car length, 1.51 m body top, and track/spine underside are included. The actual renderer's imported body bounds remain checked by the engine automation test.

The final rotating envelope is swept continuously against the same triangles used to render the terrain. Ground certification covers the full box interior and between-sample motion. Support tests account for actual flat caps before conservative contact tests, so a footing does not acquire an imaginary hemispherical top. Foundations may be partly buried on a slope: their lower caps anchor below soil, while all exposed steel is independently certified against the actual terrain triangles. Forcing the entire wide footing top above the uphill soil incorrectly raised its downhill edge into low track. Steel uses an oriented solid enclosure when a coarse height bound is inconclusive.

The model still needs modern population/vehicle calibration, restraint kinematics, manufacturing/installation tolerances and a licensed review of all applicable F2291-26 requirements before any real ride-engineering claim. These limitations are reported; no additional blanket height or force cap is presented as an ASTM requirement.

Sources:

- [ASTM current F2291 catalogue](https://store.astm.org/standards/f2291)
- [Accessible reproduction of F2291-25](https://studylib.net/doc/27870841/f2291-25), clauses 3.1.13, 3.1.15 and 6.6.3–6.6.4.
- [NASA primary handbook PDF](https://ntrs.nasa.gov/api/citations/19830009967/downloads/19830009967.pdf), PDF p.139 / printed p.142.
- [Intamin Namazu](https://www.intamin.com/project/namazu/) confirms deliberate ground-hugging landscape design; it does not publish a universal clearance dimension.

## Escarpment delivery terrain placement

The ordinary rail datum is2m. The generated summit uses a3.5m rail-height intent for its local grade controls; rasterization and lateral envelope clearance produce roughly3.1m winding-section means in the baseline. That is terrain placement, not an added clearance limit. Nearby lower passages constrain the same broad landform. The final full-box and support certification uses the actual shared8m triangles with zero configurable external gap. The old isolated knoll type remains only for compatible saved profiles.
