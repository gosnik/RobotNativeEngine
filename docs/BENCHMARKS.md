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

The Rapier backend builds one sorted entity snapshot per `sync_from_ecs` call
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
