# agents.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Repair the Interacting Design

**Improve adjacent agent-authored code when it makes the whole system simpler and correct.**

- Review the whole interacting design. Restructure related code, consolidate conflicting owners and remove obsolete paths; do not preserve a broken architecture merely to keep a diff small.
- Fix the underlying physical or software constraint. Do not stack compensating branches, retries, clamps, fallback paths or duplicated calculations around a failing design.
- Prefer one clear owner for each calculation and one coherent data flow. Remove implementations and tests made obsolete by the change. Extract a helper only when it removes real duplication or makes a meaningful contract independently testable.
- Reduce unnecessary code and control flow where justified. Do not chase a line quota, compress readable code, remove useful tests or move complexity between files merely to claim deflation.
- Earlier agent-authored adjacent code is eligible for repair. Preserve unrelated user work and useful frozen evidence, and verify the behavior affected by every cleanup.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

## Project-specific engineering and maintenance

Continue the existing native game and preserve unrelated work and frozen evidence.

These requirements apply to the entire codebase and every future edit and commit:

- State the intended behavior and verify it with relevant tests and actual evidence before committing. Distinguish changed fixture geometry from weakened acceptance. Report remaining failures honestly.
- Ground coaster geometry, operation placement, forces and clearance in real designs and verified sources. Distinguish terrain clearance, vehicle/hardware sweeps and patron reach/containment envelopes. Public summaries are not a complete F2291 compliance basis.
- Preserve the selected targets, physical force/power limits, exact-version saves and mandatory 960/1920 Hz acceptance. Never fabricate benchmark recordings or present a rejected ride as accepted.
- Review the final diff for redundant loops, conflicting rules, unused paths and avoidable state. Keep commits coherent, run relevant CI and preserve failed experimental evidence separately from promoted results.

## Workflow and Completion Evidence

Rules 1, 2 and 4 retain the [Karpathy-inspired guidelines](https://github.com/multica-ai/andrej-karpathy-skills/blob/main/CLAUDE.md). Rule 3 is the user's explicit replacement for the upstream restriction on adjacent changes.

- Apply the relevant [Superpowers workflows](https://github.com/obra/superpowers): explicit requirements, root-cause investigation, failing reproductions before bug fixes, review against requirements, code-quality review and fresh verification before completion claims. User instructions govern scope, worktree/agent choices and already-authorized actions.
- Use Graphify's local code graph to guide ownership and impact review. Verify graph relationships against source; graph connectivity is not proof of a bug or correctness. Exclude frozen snapshots, generated output, dependencies and art from the active-source index.
- Continue the correction and verification loop until every required gate is satisfied. Bind final evidence to the tested source, binaries and saves. Report unresolved blockers honestly; confidence, partial passes and historical green snapshots are not completion evidence.
