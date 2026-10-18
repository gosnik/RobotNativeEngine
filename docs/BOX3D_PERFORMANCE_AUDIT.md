# Box3D performance audit

This audit compares the RNE wrapper and its vendored Rapier implementation with
[Box3D at efca0c1](https://github.com/erincatto/box3d/tree/efca0c184d40fd6a82016983f1601c4d23965867).
Candidates are kept only after correctness checks and measured evidence; each
accepted implementation is committed separately and integrated into Hexapod's
pinned engine checkout. Exhaustion means the identified candidates have a
documented disposition, not a proof that no future optimization exists.

## Completed before this audit pass

- Shared, retained sorted entity snapshot: `46a28d2`; 55.4% less synchronization
  time and 43.3% less complete-tick time in the mixed-entity benchmark.
- Removed duplicate post-step query rebuild: `f4a660e`; 4.75% less physics-step
  time and 4.17% less complete-tick time in the moving-collider benchmark.
- Reused writeback buffers and a shared joint list: rejected after five
  alternating runs showed less than 1% focused improvement. The implementation
  was discarded.

These percentages describe different fixtures and must not be added. Commands
and fixture details are in [BENCHMARKS.md](BENCHMARKS.md).

## Candidate inventory

| Technique | Evidence and disposition |
| --- | --- |
| Dense work subsets | Accepted: select physics-relevant ECS archetypes before deterministic sorting. Against `f4a660e`, synchronization took 86.1% less time and complete ticks 53.7% less in the mixed-entity fixture. New archetypes and malformed standalone controls have regression coverage. |
| Reuse sensor queries | Box3D `sensor.c` retains per-worker sensor data. RNE LiDAR repeats its central ray when sampling a beam footprint; reuse is under measurement. |
| Dirty spatial updates | Box3D `broad_phase.c` tracks moved proxies. Rapier already updates moved colliders; remaining pre-step rebuilds need a lifecycle audit. |
| Force bookkeeping | Box3D uses membership flags/bitsets instead of repeatedly searching work lists. RNE's forced-body list and effort buffers remain candidates. |
| Transform and joint metadata reuse | Box3D prepares invariant solver data before repeated work. RNE's hierarchy walks and joint degree-of-freedom offsets remain candidates. |
| SIMD | Rapier's optional `simd-stable` feature passes the 75-test baseline suite. A 256-contact sliding-box probe measured 508.073 -> 354.229 microseconds per step with identical final state hashes across five alternating runs. Hexapod compatibility remains to be evaluated before enabling it. |
| Parallel solver | Vendored parallel solver module wiring is commented out. Enabling the Cargo feature alone is not an implementation; enabling it requires a separate solver/telemetry compatibility project. |
| Sleeping | Rapier already owns active islands and sleeping. Skipping wrapper updates merely because a body sleeps could suppress commanded motion or forces; no such shortcut is accepted. |
| Contact buffers | RNE already clears and reuses contact/event vectors, and exposes borrowed slices. Hexapod consumes current point-level contact data; suppressing publication would change behavior. |
| Batch raycasts | Existing trait fallback preserves batch ordering. A native override can only help wrapper overhead unless traversal or result ownership changes; evaluate separately from the LiDAR duplicate-query candidate. |

## Validation environment

Measurements use release builds on an Intel Core Ultra 7 270K Plus, with
Rust 1.98.1 because the repository-pinned 1.95.0 installation is unavailable.
Before/after builds use explicit manifests and distinct target directories;
timings alternate between versions after compilation finishes. Kernel policy
(`perf_event_paranoid=4`) prevents hardware-counter profiling, so this audit
uses timed fixtures and operation counts instead.

The existing workspace Clippy gate reports `chunks_exact_to_as_chunks` in
untouched code under this newer toolchain. Targeted tests and headless CI are
reported separately from that gate. Hexapod integration verifies its engine
pin, viewer-enabled release build, and an explicit-snapshot headless run.
