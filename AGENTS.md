# VibeCoaster working entrypoint

Read README.md, HANDOFF.md and docs/CODE_MAP.md. V3 route compilation is the
active source; default.3 handoffs and checkpoint receipts are historical.

Keep ride intent, construction and independent validation separate. Validation
must not repair geometry, adjust forces or silently retry another ride. Inherit
the complete geometry and frame state at boundaries; do not insert arbitrary
upright/flat resets. Plan hardware with the route and evaluate actual replay.

Preserve the intentional rest-to-180-km/h launch in about 1.4 seconds, independent
physics, occupied-envelope clearance and fresh saved-design checks. Prefer a few
substantive experience regressions over track snapshots or large matrices.
Passing checks do not prove visual quality or reference fidelity.

Build using native/CMakeLists.txt with at most eight compiler workers. New core
translation units also need Unreal Private/Core*.cpp wrappers. Historical saves
retain their recorded identity; new generation uses native/core/include/coaster/version.hpp.
Do not mistake a loaded old profile for output of the current generator.
