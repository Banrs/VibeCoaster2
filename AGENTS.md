# Engineering and maintenance

Continue the existing native game and preserve unrelated work and frozen evidence.

These requirements apply to the entire codebase and every future edit and commit:

- Fix the underlying physical or software constraint. Do not stack compensating branches, retries, clamps, fallback paths or duplicated calculations around a failing design.
- Prefer one clear owner for each calculation and one coherent data flow. Remove implementations and tests made obsolete by the change. Extract a helper only when it removes real duplication or makes a meaningful contract independently testable.
- Reduce unnecessary code and control flow materially where justified. Do not chase a line quota, compress readable code, remove useful tests or move complexity between files merely to claim deflation.
- State the intended behavior and verify it with relevant tests and actual evidence before committing. Distinguish changed fixture geometry from weakened acceptance. Report remaining failures honestly.
- Ground coaster geometry, operation placement, forces and clearance in real designs and verified sources. Distinguish terrain clearance, vehicle/hardware sweeps and patron reach/containment envelopes. Public summaries are not a complete F2291 compliance basis.
- Preserve the selected targets, physical force/power limits, exact-version saves and mandatory 960/1920 Hz acceptance. Never fabricate benchmark recordings or present a rejected ride as accepted.
- Review the final diff for redundant loops, conflicting rules, unused paths and avoidable state. Keep commits coherent, run relevant CI and preserve failed experimental evidence separately from promoted results.
