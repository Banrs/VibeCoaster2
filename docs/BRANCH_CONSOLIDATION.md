# Branch consolidation, 1 October 2026

Default.3 is the latest implementation and the authoritative foundation for
this reconciliation. The draft records both branch histories, preserving the
current implementation wherever the older rewrite conflicts with it.

## Active foundation

Foundation: `main` at `66000ec2dc169a7ae9660a4f3f3481badab16a0a`.
This is the latest local-source and portable-diagnosis push. Its parent is
`966a04756d614497b706adec86e28889ee568fbf`, the former
`codex/legacy-authoring` tip. That history already contains
`a2149dfb0c91a0264ae70bd49ff74bba1073f930`, the former
`codex/default-generator` tip. Those two branches' commits do not need to be
cherry-picked again. Their branch names are no longer present on the remote;
their commits and history are retained in `main`.

The active product remains default.3, with the later support, camera, train-art
proposal and diagnostic-source work. `HANDOFF.md` carries the current open
design requirements. Source consolidation does not approve the ride, adopt
the proposed train into the runtime, or restore rejected default.4 behaviour.
The canonical save and separate adapted-support review remain distinct.

## Preserved alternative

`codex/fresh-rewrite` at `15781504dbd0624ce2d461aceefca3636608c864`
is a separate implementation. It shares the `a2149df` ancestor but has nine
unique commits; current `main` has sixteen commits beyond that ancestor.
The reconciliation commit has both tips as parents. It deliberately retains
current default.3 source over the incompatible replacement implementation;
including the rewrite's history does not mean adopting every rewrite feature.
The original rewrite branch remains intact for reference.
The existing [rewrite PR #3](https://github.com/Banrs/VibeCoaster2/pull/3)
is superseded by this separate reconciliation draft. Review and merge the
reconciliation draft rather than the older replacement proposal. Neither
existing branch is changed while this draft is under review.

A dry three-way merge exposes 81 unique conflicted paths, including
modify/delete and rename conflicts across the native core and Unreal source.
The text conflicts are `.gitattributes`, `.github/workflows/native.yml`,
`.gitignore`, `README.md`, and `docs/acceleration-standard.md`. Resolving only
those text markers would not make the two implementations compatible.

| Area | Resolution and reason |
| --- | --- |
| Build and source layout | Retain `native/` and its current CMake targets. The rewrite replaces them with root-level `core/` and `unreal/` and different APIs. |
| Geometry, train and terrain | Retain default.3 and the latest source/evidence. The rewrite's independent recipe/site and six-car train assumptions do not match the current seven-car/fourteen-seat proposal and saved baseline. |
| Persistence | Preserve current saved-design validation and both hash-verified diagnostic saves. Rewrite format 3/site revision 2 is a separate format, not a migration for these saves. |
| Runtime and launch | Retain current camera controls, native Unreal integration, Play shortcut and package/profile identities. Rewrite replaces the runtime and launcher. Its old GPU/package receipts do not validate this source. |
| Numerical limits | The shared acceleration evaluator and its boundary tests are already retained (differences are formatting only). Keep current independent physics checks; do not import conflicting historical force allowances or call old branch evidence current verification. |
| Repository metadata | Preserve current binary/evidence attributes and ignored native Unreal output. The rewrite's narrower generated cook-log ignore is already covered here. |

## Compatible build fix

The rewrite's CMake configuration explicitly enables MSVC C++ exception
unwinding with `/EHsc`. Carry that requirement into the current native target
and its consumers, while retaining strict floating-point settings and C++20.
Remove the Windows workflow's blanket `CMAKE_CXX_FLAGS` override, which dropped
CMake's normal exception flags.

This addresses the current `main` failure, not a historical rewrite failure:
[run 36838739764](https://github.com/Banrs/VibeCoaster2/actions/runs/36838739764)
passed on macOS but failed the Windows `support_system` rollback test. Its
compiler emitted warning C4530: exception handlers were compiled without unwind
semantics. The support-generation rollback relies on destruction of its local
restore guard during cancellation. Tests must verify the complete previous
support graph survives cancellation, rather than only counting attachments.

## Verification and follow-through

Run the native build, complete CTest suite and fresh validation of both bundled
saves. Check Windows and macOS CI on the exact proposed commit before merging.
Native checks do not establish Unreal runtime behaviour, manual interaction,
visual approval, package readiness or resolution of the design complaints.

Keep `codex/fresh-rewrite` for reference. The history reconciliation does not
carry over its replacement runtime or physics. Any later port of its motion kernel,
saved format, operating-scenario/refinement validation, hardware model or GPU-ready scene
transaction needs an explicit compatibility design and current evidence against
the retained baseline. Its historical timing numbers are not transferable.
No source branch is deleted and no existing branch is force-updated by this work.
