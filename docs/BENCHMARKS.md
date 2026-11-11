# Deterministic benchmark reports

The benchmark aggregator combines the existing physics-conformance and
scenario-scale reports without reimplementing either producer:

```text
cargo run --locked -p xtask -- benchmark
```

Missing producer reports are generated automatically. Use `--no-generate` to
require reports that already exist, or pass `--physics-report`,
`--scenario-report`, and `--output` to select explicit paths. The stable report
is written to `artifacts/benchmarks/report.json` by default.

The schema-v1 artifact has kind `rne_benchmark_report`. Its cases and evidence
are sorted, evidence files are recorded with repository-relative paths and
SHA-256 digests, and `content_digest` is the SHA-256 of canonical JSON with
the digest field itself removed. Each case records the exact nanosecond
`fixed_delta_ticks` used by `SimDuration`, plus its derived explicit-unit
`fixed_delta_s` projection (`simulation_hz` uses rounded-rate integer
division; seconds use nearest-nanosecond rounding). It contains no timings,
host metadata, timestamps, stdout, or absolute paths. Each case embeds the
applicable `rne_core::DeterminismContract`.

Timing data is deliberately separate and optional:

```text
cargo run --locked -p xtask -- benchmark --timings
```

This writes `artifacts/benchmarks/timings.json`; use `--timings-output` for a
different location. Timing and host fields in that artifact are volatile and
must not be used to compare stable benchmark report bytes.

For the full capability, conformance, benchmark, and Failure Capsule gate, run
`cargo run --locked -p xtask -- evidence` as documented in
[EVIDENCE_QUICKSTART.md](EVIDENCE_QUICKSTART.md).

## Rapier synchronization scratch storage

The Rapier backend builds one sorted physics-relevant entity snapshot per `sync_from_ecs` call
and shares it across body, joint, attachment, motor, and diagnostic passes.
Its backing allocation is retained between calls. The snapshot is rebuilt on
every call so spawning, despawning, and component changes remain visible; it
is not a persistent topology cache. Non-body entities also skip world-transform
composition in the body pass.

This applies the scratch-storage reuse technique used by Box3D's
[`b3ArenaSync` and `b3GrowStack`](https://github.com/erincatto/box3d/blob/efca0c184d40fd6a82016983f1601c4d23965867/src/arena_allocator.c):
retain capacity for repeated simulation work rather than repeatedly allocating
temporary storage. RNE uses safe Rust vectors rather than copying the C arena
implementation. Entity ordering, solver settings, time steps, and public
physics interfaces are unchanged. This optimization targets ECS synchronization;
it does not imply an equivalent improvement in collision detection or solving.

Run the public-API timing probes serially in a release build:

```sh
cargo test -p rne_physics_rapier --release --test sync_benchmark -- --ignored --nocapture --test-threads=1
```

For a before/after comparison, copy the same `tests/sync_benchmark.rs` into the
baseline crate, use explicit `--manifest-path` arguments and separate target
directories, and alternate runs after compiling both builds. The fixture has
128 separated bodies, zero gravity, and 4,096 non-physics entities, with eight
warmup calls and 128 measured calls. It deliberately stresses synchronization
overhead; it is not a contact-heavy robotics or Hexapod benchmark.

On an Intel Core Ultra 7 270K Plus with Rust 1.98.1, five alternating release
runs against baseline `f614389425a1e7d7005fbf188226a99dacb82f6b` measured a
median `sync_from_ecs` time of 251.637 microseconds before and 112.330
microseconds after (55.4% less time). The complete headless update
(`sync_from_ecs`, physics step, and `sync_to_ecs`) measured 322.268 microseconds
before and 182.696 microseconds after (43.3% less time), also the median of
five alternating runs. Wall-clock results vary by host and load.
The repository-pinned Rust 1.95.0 toolchain was unavailable on this host.

## Incremental post-step queries

`advance_rapier_pipeline` passes the backend's query pipeline to Rapier's
`PhysicsPipeline::step`, which updates it incrementally after advancing collider
positions. The backend therefore does not perform another full QBVH rebuild
immediately afterward. The full update after `sync_from_ecs` remains necessary
for externally changed collider state. A regression test queries a moving body
immediately after stepping, before any further ECS synchronization.

This follows Box3D's use of tracked moved proxies in
[`b3World_Step`](https://github.com/erincatto/box3d/blob/efca0c184d40fd6a82016983f1601c4d23965867/src/physics_world.c)
to update its spatial data, instead of redundantly rebuilding it. RNE continues
to use Rapier's query implementation and does not change collision or solver
settings.

The `step_with_many_moving_colliders` and
`headless_step_with_many_moving_colliders` ignored tests in `sync_benchmark.rs`
measure 1,024 moving colliders. The separate regression test checks post-step
raycast freshness and hit distance. Run the timing probes with the
same release command above and compare against the preceding synchronization
optimization (`46a28d2`), using the same fixture in both checkouts.

On the same CPU and Rust 1.98.1 release configuration, five alternating runs
measured median physics-step time of 432.712 microseconds before and 412.142
microseconds after (4.75% less time). Median complete-tick time fell from
591.904 to 567.216 microseconds (4.17% less time). The benchmark uses eight
warmup steps and 64 measured steps at 60 Hz, zero gravity, and separated moving
boxes. These are incremental gains over `46a28d2`, not percentages to add to
the earlier benchmark's results or predictions for Hexapod.

## Physics-relevant archetype selection

Following Box3D's dense solver work sets, the backend now selects only archetypes
containing body, joint-description, armature, actuation, or passive-dynamics
components before sorting the shared snapshot. Component IDs and archetypes are
rechecked each call, so new components/archetypes and standalone malformed
controls remain visible. Unrelated rendering/gameplay entities skip all six
physics passes without changing the order of relevant entities.

Against `f4a660e`, five alternating release runs of the same 128-body plus
4,096-background-entity fixture measured median synchronization time of
112.335 -> 15.585 microseconds (86.1% less), and complete-tick time of
179.451 -> 83.173 microseconds (53.7% less). The before/after synchronization
ranges were 111.179–114.090 and 15.546–15.773 microseconds. This improvement is
largest in scenes with many non-physics entities; it is not a Hexapod-specific
timing claim.

## Reusing LiDAR's central beam query

For a divergent beam with multiple footprint samples, `evaluate_ray` already
has the central hit list. Footprint sample zero now borrows that list instead
of casting the identical ray again. Divergent samples, RNG slots, range
filtering, weighting, and scan failure handling are unchanged. Single-sample
and zero-divergence scans keep their existing behavior.

```sh
cargo test -p rne_sensor --release --test lidar_benchmark -- --ignored --nocapture --test-threads=1
```

This explicitly synthetic immutable-raycaster fixture performs 64 rays with
16 beam samples. It now makes exactly 1,024 queries instead of 1,088, removing
64 queries and their result allocations (5.9%). Five alternating release runs
measured median scan time of 63.717 -> 57.845 microseconds; timing ranges overlap,
so the deterministic query-count reduction is the primary evidence. The regular
beam-footprint regression checks the exact new query count and mixed-edge result.
This change benefits LiDAR callers; the current Hexapod contact simulation does
not itself scan LiDAR.

## Structural query invalidation

Following Box3D's moved-proxy bookkeeping, synchronization now rebuilds the
query tree only after collider insertion/removal. Ordinary simulation steps
already update the tree incrementally. The dirty flag survives failed syncs.
Regression coverage includes direct queries after removal, owned collider
replacement with a reused entity index, and failed-sync recovery. This also
fixes an existing early return that skipped removal of ordinary ECS colliders.

Against `7201043`, five alternating release runs measured mixed-scene sync
15.551 -> 10.896 microseconds (29.9% less) and full ticks
82.619 -> 78.349 microseconds (5.2% less). Sync ranges were 15.365–15.762
and 10.798–11.032; full-tick ranges were 82.342–82.867 and 78.238–79.104.
The 1,024-moving-collider full-tick median was 542.906 -> 519.932 microseconds
(4.2% less), with overlapping ranges 534.712–574.596 and 512.205–545.840.
These are the existing `sync_benchmark` fixtures; 80 all-feature backend tests
passed. No Hexapod-specific speedup is inferred from these scene timings.

## Linear direct-effort cleanup bookkeeping

Direct efforts now append bodies to the ordered cleanup list without searching
the entire list first. Rapier's force/torque reset is a no-op after the first
reset, so repeated entries preserve results; external wrenches already use this
append policy. This removes quadratic wrapper bookkeeping for large joint sets.

Run `cargo test -p rne_physics_rapier --release --test effort_benchmark --
--ignored --nocapture`. The fixture has 18 or 256 independent impulse joints,
16 warmup ticks and 256 measured ticks at 1 kHz. Five alternating runs against
`fa71d4b` measured 256-joint sync 89.606 -> 67.310 microseconds (24.9% less),
and complete ticks 672.521 -> 651.851 microseconds (3.1% less; ranges overlap).
Sync ranges were 89.278–90.015 and 66.549–68.606. At 18 joints, sync medians
were 6.519 -> 6.447 and full ticks 57.326 -> 57.167 microseconds; differences
are within noise, so no small-robot speedup is claimed. All five final hashes
matched across implementations at both sizes. The backend tests passed.

## Reusing completed-effort buffers

Swapping the pending/completed effort maps and clearing the old completed map
retains their allocations while preserving one-step publication. The feedback
regression also advances three times without another command and checks that
old measurements never reappear. This applies the general scratch-reuse
principle; Box3D does not have this robot-specific feedback lifecycle.

Against `af59fb9`, five alternating runs of `effort_benchmark` measured 18-joint
sync 6.360 -> 5.968 microseconds (6.2% less) and 256-joint sync
66.738 -> 63.032 microseconds (5.6% less). Sync ranges were 6.286–6.380 versus
5.884–5.993, and 65.700–66.955 versus 61.350–63.710. Complete-tick ranges
overlap at both sizes (medians 56.972 -> 57.391 and 649.576 -> 639.065),
so the claim is reduced allocation/synchronization work rather than a robust
end-to-end speedup. Final state hashes matched at both sizes in every run.

## Skipping unused physics-owned transforms

Existing `PhysicsOwnedPose` bodies no longer walk ECS parent chains and convert
isometries whose results were discarded. Initial body creation and ordinary
ECS-driven bodies still compute their transforms. Collider synchronization is
unchanged. Run `cargo test -p rne_physics_rapier --release --test
owned_pose_benchmark -- --ignored --nocapture`.

Against `26b6b88`, five alternating runs with 128 bodies measured depth-one
sync 14.054 -> 9.207 microseconds (34.5% less; ranges 13.935–14.298 and
9.007–9.303), and depth-sixteen sync 55.085 -> 9.237 microseconds
(83.2% less; ranges 54.174–55.141 and 9.131–9.277). This deliberately isolates
unused hierarchy walks; it is not a full-tick or Hexapod speedup claim.
Backend tests, including physics-owned pose behavior, passed.

## Avoiding unused joint offsets

In builds without experimental armature, synchronization computes a joint's
assembly offset only when an explicit passive-damping component needs it.
Armature builds and joints with explicit damping retain the existing calculation
and validation order. This is local elimination of unused metadata work, not
a Box3D multibody algorithm.

Run `cargo test -p rne_physics_rapier --release --test joint_metadata_benchmark
-- --ignored --nocapture` with default features. Against `4fad9d4`, five
alternating runs measured 128-link sync 19.515 -> 14.528 microseconds
(25.6% less; ranges 19.310–19.748 and 14.232–14.793). At 18 links, medians
were 2.165 -> 2.148 microseconds, within noise. The fixture deliberately omits
explicit passive damping; imported robots that supply damping, including
Hexapod, do not benefit from this shortcut. Default and all-feature backend
tests passed.

## Opt-in SIMD contact solving

Box3D's wide contact batches motivate exposing Rapier's existing stable SIMD
solver through `rne_physics_rapier/simd`. This feature remains opt-in: scalar
and vectorized arithmetic need not agree on arbitrary scenes, and Rapier
rejects combining SIMD with `enhanced-determinism`. Keep feature configuration
and target environment fixed for replay comparisons.

```sh
cargo test -p rne_physics_rapier --release --test contact_benchmark -- --ignored --nocapture
cargo test -p rne_physics_rapier --release --features simd --test contact_benchmark -- --ignored --nocapture
```

Build these into separate target directories before timing. The public-API
fixture has 256 frictionless boxes sliding on a floor, 16 warmup steps and
128 measured steps at 60 Hz. Five alternating runs measured scalar
518.910 versus SIMD 362.659 microseconds per physics step (30.1% less).
Ranges were 510.221–531.297 and 352.922–376.851. Every run had 256 contacts
and final state hash `eeab673517191700`. The full 80-test all-feature backend
suite passed with SIMD enabled. This fixture does not establish a Hexapod
performance gain.

Hexapod compatibility was checked with three alternating scalar/SIMD pairs
each of two-second hold and walk runs using the same immutable model and
configuration. All non-timing result fields and joint, walking, and stance
telemetry matched exactly; all runs were finite with zero faults and no falls.
Physics-step medians were 2.040 versus 2.043 seconds for hold and 2.030 versus
2.029 seconds for walk, with overlapping ranges. Thus SIMD is compatible in
these smoke scenarios, with no demonstrated robot-specific speedup. These
short runs are not a gait qualification.
