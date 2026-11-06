# Faultline: creative direction and current authoring

6 November 2026. The user rejected the first rewrite's ride and requested
creative liberties. This note records the resulting
implemented composition and source parameters. Actual generated results and
remaining review limits are recorded in the [Faultline verification](../diagnostics/faultline/README.md).
The terrain brief below remains distinct from verified geometry and native checks.

The earlier accepted forward seed 42 preserved the old sequence: 1,402 m /
29.91 s opening, giant camelback, raised return, full loop and Immelmann. It
retained 23 front-seat force beats and roughly 33 seconds of airtime across
8,682 m / 183.13 s. Its return-to-stop grew from 24.13 to 31.63 s. Faster
generation had not given the ride a new identity. The historical
[comparison](../diagnostics/rewrite/seed42-comparison.svg) documents that ride,
not the new Faultline composition.

## Three acts with different jobs

The spatial brief is a ravine opening into a cliff amphitheatre. The opening
stays low, the cliff owns the major skyline reveal, and the return uses speed
and changing terrain proximity instead of another catalog of giant elements.
The following numbers are **authoring inputs**, not measured heights or forces.

| Act | Current source direction | Experience it is intended to create |
| --- | --- | --- |
| **1. Escape the fissure** | Retain rest to 180 km/h in about 1.4 s. A light +0.3 g crest uses +45° bank and a nominal 35 m rise, resolved by seed within 28–40 m. The route folds through a ravine descent, a 240 m settling run and low exit. | Immediate launch impact, one release and a fast terrain passage. Reserve the largest visual reveal for later; remove the old 170 m opening skyline and repeated preliminary hills. |
| **2. Lose the horizon** | A 235 m cliff approach reaches one 420 m rim sweep, then the brake approach and cliff dive. | One sustained rim reveal with an obvious destination. Remove the repeated summit direction changes and extended high-altitude touring. |
| **3. Fly out, cut back** | A 285 km/h boost feeds an original hill profile: 105 m nominal rise, seed-resolved within 95–115 m, with −0.85 g release, 18° twist and 3.8 g loading targets. A 550 m wall turn uses −65° heading change, 55 m nominal rise and 115° crest bank; its physical crest-load intent is 1.8 g. The original 688 m finale has one float, trench descent and low countersteer, leading directly to a short station closure. | A large floating flight, one strong change of direction and a low terrain return. The default removes the full loop, Immelmann and extra return crest. |

The first brief proposed a half-loop/roll as the turnaround. At the developing
route's roughly 72 m/s entry, the compact 100 m version demanded excessive
loading in the attempted construction. Enlarging it toward a 175 m inversion
would restore the giant-element catalog the redesign was meant to escape.
The chosen overbanked wall turn is an original alternative: a broad, plunging
change of direction rather than an inversion retained for its own sake. Its bank and rise remain subject to actual seat
histories and geometry, not assumptions about what a single angle implies.

The opening's headings progress to −75°, then −175°, then −55° relative to its
entry. Its light positive crest avoids the earlier bank-polarity reversal. The
new camelback uses a generic authored hill with force and twist intent,
rather than a scaled copy of `approved-camelback-v1`. The finale also replaces
the repeated hill/roll sequence and needs no automatic staging connector ahead
of it. These changes alter feature identity and rhythm. Absolute roll and long
neutral gaps can improve while rolling occupies a greater fraction of a much
shorter course; those are different measures.

## What remains a brief, and what remains required

Use one connected rim and an open ravine, with terrain shoulders supplying
concealment and reveal. A visible return beneath earlier track could strengthen
the composition, but no underpass beneath the outbound camelback is claimed
until actual geometry and occupied-envelope clearance establish it. Likewise,
the original length/time budget was a design aid, not a delivered result or
an exact regression requirement.

The recipe grammar now allows optional, reordered acts and a generic hill in
the camelback role. The explicit recipe remains fixed; seed-derived shape
parameters are resolved separately and saved with the generated design.
Historical recipes and saves retain their identity. The
forward compiler, complete boundary state, independent finite-train energy,
occupied-envelope clearance and duration/axis/history assessment remain.
Nominal force targets describe the desired sensation; they do not replace
those independent checks. Future hardware does not imply greater human
physiological tolerance. The explicit launch timing remains.

Judge the generated ride by its actual plan, elevation, terrain relationships
and rider passage. Accept losing cumulative airtime or force-beat count when
the result has clearer contrast and fewer repeated gestures. Existing focused
regressions should protect physical behavior; another large matrix cannot
establish that this composition is compelling.

## Sources

[Intamin's Falcon's Flight description](https://www.intamin.com/project/falcons-flight/)
supports cliff reveals and intensity/recovery contrast.
[B&M's Tormenta gallery](https://www.bolliger-mabillard.com/coasters/tormenta-rampaging-run)
informed the earlier inversion study; it does not require retaining inversions
in this default. [NoLimits' FVD manual](https://www.nolimitscoaster.com/nolimits2/help/pages/forcevector.html)
supports force/roll authoring with resulting speed. These sources do not
prescribe Faultline's geometry. See the
[research record](generator-rewrite-research.md) for verified facts and limits.
