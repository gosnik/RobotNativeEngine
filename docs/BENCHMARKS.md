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
