# Paused Riftwake ride work

The user paused train and ride-layout updates to prioritize a grand high-end station and the track, support, terrain, LSM and brake models. No new ride was accepted or promoted.

`08-riftwake-paused.patch` preserves the unfinished authoring, physics, pacing and version edits against Git checkpoint96c102f. Its application was checked with `git apply --check` after restoring the baseline. Exact source copies and SHA-256 checksums also remain locally in `out/paused-riftwake-layout/`. The active generator is back on the accepted Default2 source; asset work continues independently.

Last completed diagnostic: `out/riftwake-fourth-*`,195.37s and300.53km/h, closed but rejected for clearance, force-history, shape and pacing findings. Latest sixth diagnostic rejects at Wave authoring. Physics subsequently established that Wave exit3.0g (instead of current1.5g in the paused patch) constructs and connects to the140m-reference loop with3.8g ascent/recovery and1.2g crest. That final prescription was not applied because the user paused the work.

On an explicit resume, apply the patch carefully, make that source correction, and perform one focused generation/review. Do not treat a completed replay as accepted. Keep the original accepted track and saved profile until all new checks pass. The1.4s launch was explicitly retained by the user.
