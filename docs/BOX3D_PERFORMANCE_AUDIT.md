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
| Reuse sensor queries | Accepted: reuse LiDAR's central hit list for footprint sample zero. A 64-ray, 16-sample scan consistently avoids 64 queries and their result allocations (1,088 -> 1,024). Synthetic timings are documented separately from this exact operation-count result. |
| Dirty spatial updates | Accepted: rebuild the pre-step query tree only after collider insertion/removal. Rapier incrementally maintains moving proxies during stepping. Lifecycle regressions cover removal, same-index replacement, and recovery after failed synchronization; mixed-scene sync fell 29.9% and full ticks 5.2%. |
| Force bookkeeping | Accepted: eliminate quadratic membership scans in direct-effort cleanup, using Rapier's idempotent resets and preserving append order. 256-joint sync fell 24.9%; 18-joint timing was unchanged within noise. Accepted separately: retain alternating effort-map allocations, reducing measured sync by 6.2% at 18 joints and 5.6% at 256 joints; full-tick timing overlaps. This is a wrapper-specific application of avoiding repeated work, not a port of Box3D's force lifecycle. |
| Transform and joint metadata reuse | Box3D prepares invariant solver data before repeated work. RNE's hierarchy walks and joint degree-of-freedom offsets remain candidates. |
| SIMD | Rapier's optional `simd-stable` feature passes the 75-test baseline suite. A 256-contact sliding-box probe measured 508.073 -> 354.229 microseconds per step with identical final state hashes across five alternating runs. Hexapod compatibility remains to be evaluated before enabling it. |
| Parallel solver | Vendored parallel solver module wiring is commented out. Enabling the Cargo feature alone is not an implementation; enabling it requires a separate solver/telemetry compatibility project. |
| Sleeping | Rapier already owns active islands and sleeping. Skipping wrapper updates merely because a body sleeps could suppress commanded motion or forces; no such shortcut is accepted. |
| Contact buffers | RNE already clears and reuses contact/event vectors, and exposes borrowed slices. Hexapod consumes current point-level contact data; suppressing publication would change behavior. |
| Batch raycasts | Not pursued: a native override only hoists the world lookup; the sensor raycaster abstraction has no batch entry point, and Box3D provides no transferable batch traversal primitive. Meaningful batching requires a separate sensor/query API design with error and order conformance tests. Central-ray reuse already removes actual traversal/allocation work without that change. |

## Solver internals inspected

Rapier already provides active islands, warm starting, contact scratch reuse,
and incremental moving-collider spatial updates. These are not new gains to claim.
In `Multibody::update_inertias`, nalgebra's six-dimensional `quadform` workspace
is stack storage, so replacing it would not remove a heap allocation. The two
`LU::new` calls do allocate owned matrices, but nalgebra exposes no drop-in
refactorization API with reusable storage. Replacing factorization arithmetic
and serialized solver state requires a separately validated numerical solver
project; no speculative change is accepted here.

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
