# Native validation

[Current v083 qualification](V083_INTEGRATION.md) is the single owner of local results and source/binary/save provenance. [NEXT_CHAT.md](NEXT_CHAT.md) is the continuation entry point. Source64 has passed all required local gates after the final dead-branch cleanup. [Promotion records](artifacts/flow-intent-v083-20260910/integration-20260911/promotion-64/) and GitHub checks record integration and main status.

Use [tools/BUILDING.md](tools/BUILDING.md) for focused and full test commands. Run affected component checks before broad generation; CMake labels separate `component` and `integration`. A focused pass does not replace the complete native, saved-replay, convergence, generality and CI gates.

Final qualification requires all 36 CTest suites in Release, components before integration, two workers, existing timeouts and `--stop-on-failure`; all 44 unchanged requests; the eight required saved-ride audits; Python and portable wrappers; and ten relevant real UE contracts. UE automation status comes from its report counts, not the process exit code alone. Source64 provenance binds 168 inputs and 65 binaries, including external UE dependencies. Promotion records under `artifacts/flow-intent-v083-20260910/integration-20260911/promotion-64/` must bind green final-integration CI before merge and green main CI afterward.

Acceptance retains selected targets, actual finite-train force/power behavior, complete terrain/vehicle/hardware clearance, exact-version saves and mandatory 960/1920 Hz agreement. Rejected rides and interrupted attempts must remain labelled as such.

Public API preservation is measured from the approved source59 checkpoint; earlier experimental FVD interfaces differ from main. Motor/quiet-tail/duration comparisons and Graphify relationships are descriptive evidence, not acceptance criteria. Packaging, benchmark qualification, POV review, FPS/load measurement and Mac/Metal runtime verification are outside this milestone.

[Historical validation records](artifacts/flow-intent-v083-20260910/integration-20260911/handoff-20260912-source59/pre-cleanup/VALIDATION.md) preserve previous releases, failures and dated experiments. Those results apply only to their recorded source and binaries. [Core numerical guidance](core/NUMERICS.md) and [force scope](core/FORCE_GUIDELINES.md) remain applicable.
