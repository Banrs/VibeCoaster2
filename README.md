# VibeCoaster2

Standalone C++20 hybrid FVD/spline coaster generator and Unreal 5.8 viewer. The active design and acceptance contract is [NEXT_SESSION.md](NEXT_SESSION.md). The revised default now passes native baseline acceptance; checkpoint packages, corpus, GPU checks and performance targets are still in progress.

The current playable fallback is the verified **2.0.0-escarpment.1** build. Open **Play VibeCoaster2.lnk**; **F9** loads its saved seed 42 ride and **Space** starts or pauses it. **Enter** generates, **Tab** opens setup, **1/2/3** select front/middle/rear, **M** toggles overview, **R** restarts, and **F5** saves. `UserData-Escarpment` is its isolated profile. **Play VibeCoaster2 Highlands.lnk** preserves the earlier packaged fallback and profile. The original sibling project remains unchanged.

The Escarpment package is fallback evidence: its 300 km/h baseline reaches 180 km/h in 1.397000 s, peaks at 300.54 km/h, and completes in 201.27 s. Its reported caps are Gz −1.5…+5, Gy ±1.5, Gx ±4.5, and 20 g/s per component. See [the fallback verification report](docs/verification.md), [the evidence index](docs/checkpoints/README.md), and [the machine-readable report](docs/verification.json). Those reports describe shipped snapshots and do not replace the current contract in `NEXT_SESSION.md`.

Approved camelback image, trace, and fit data remain under [docs/references](docs/references). Historical proposals and generated experiments are indexed as evidence; they are not implementation instructions. The revised native baseline passes the implemented F2291-25 acceleration assessment described in [the standard record](docs/acceleration-standard.md). F2291-26 conformity and whole-standard certification are unclaimed.

Build the core with `native/CMakeLists.txt`. Package Unreal with `native/unreal/scripts/package.ps1`; run the representative corpus with `native/core/tests/redesign_corpus.py`.

Export the editable default with `coaster_cli recipe --out ride.vcrecipe`, then compile edits with `coaster_cli generate --recipe ride.vcrecipe --out ride.coaster`. Saves retain the recipe and authored sources; loaded geometry receives fresh validation. Full GUI authoring follows this foundation.
