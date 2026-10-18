//! Manual timing probes for deterministic Rapier ECS synchronization.
//!
//! Run with `cargo test -p rne_physics_rapier --test sync_benchmark -- --ignored --nocapture`.
//! The file deliberately uses only the public API so it can be copied unchanged
//! into a baseline checkout for an old/new comparison.

use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{Collider, PhysicsBackend, PhysicsWorldDesc, RigidBody};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_world::Transform3;
use std::time::Instant;

const PHYSICS_ENTITIES: usize = 128;
const NONPHYSICS_ENTITIES: usize = 4_096;
const WARMUP_STEPS: usize = 8;
const MEASURED_STEPS: usize = 128;
const COLLIDER_BENCHMARK_ENTITIES: usize = 1_024;
const COLLIDER_MEASURED_STEPS: usize = 64;

fn benchmark_fixture() -> (RapierBackend, rne_physics::PhysicsWorldId, World) {
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            solver_iterations: 4,
        })
        .expect("physics world");
    let mut world = World::new();

    for index in 0..PHYSICS_ENTITIES {
        world.spawn((
            RigidBody::default(),
            Collider::cuboid(Vec3::splat(0.05)),
            Transform3::from_translation_rotation(
                Vec3::new(index as f64 * 0.2, 0.0, 0.0),
                Quat::IDENTITY,
            ),
        ));
    }
    for _ in 0..NONPHYSICS_ENTITIES {
        world.spawn_empty();
    }

    (backend, physics_world, world)
}

fn fixed_step() -> SimDuration {
    SimDuration::from_hertz(Hertz::new(60.0))
}

fn collider_benchmark_fixture() -> (RapierBackend, rne_physics::PhysicsWorldId, World) {
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::ZERO,
            solver_iterations: 4,
        })
        .expect("physics world");
    let mut world = World::new();

    for index in 0..COLLIDER_BENCHMARK_ENTITIES {
        let x = (index % 64) as f64 * 0.2;
        let y = (index / 64) as f64 * 0.2;
        world.spawn((
            RigidBody {
                linear_velocity_m_s: Vec3::X * 0.01,
                ..RigidBody::default()
            },
            Collider::cuboid(Vec3::splat(0.04)),
            Transform3::from_translation_rotation(Vec3::new(x, y, 0.0), Quat::IDENTITY),
        ));
    }
    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("initial sync");

    (backend, physics_world, world)
}

#[test]
#[ignore = "manual benchmark; compares the full ECS-to-Rapier synchronization path"]
fn sync_from_ecs_with_nonphysics_entities() {
    let (mut backend, physics_world, mut world) = benchmark_fixture();
    for _ in 0..WARMUP_STEPS {
        backend
            .sync_from_ecs(&mut world, physics_world)
            .expect("warmup sync");
    }

    let started = Instant::now();
    for _ in 0..MEASURED_STEPS {
        backend
            .sync_from_ecs(&mut world, physics_world)
            .expect("measured sync");
    }
    let elapsed = started.elapsed();
    eprintln!(
        "rapier sync_from_ecs: {:?} total, {:?}/step ({} physics + {} nonphysics entities, {} steps)",
        elapsed,
        elapsed / MEASURED_STEPS as u32,
        PHYSICS_ENTITIES,
        NONPHYSICS_ENTITIES,
        MEASURED_STEPS,
    );
}

#[test]
#[ignore = "manual benchmark; compares headless end-to-end physics update overhead"]
fn headless_step_with_nonphysics_entities() {
    let (mut backend, physics_world, mut world) = benchmark_fixture();
    for _ in 0..WARMUP_STEPS {
        step_physics(&mut backend, &mut world, physics_world, fixed_step()).expect("warmup step");
    }

    let started = Instant::now();
    for _ in 0..MEASURED_STEPS {
        step_physics(&mut backend, &mut world, physics_world, fixed_step()).expect("measured step");
    }
    let elapsed = started.elapsed();
    eprintln!(
        "rapier headless step: {:?} total, {:?}/step ({} physics + {} nonphysics entities, {} steps)",
        elapsed,
        elapsed / MEASURED_STEPS as u32,
        PHYSICS_ENTITIES,
        NONPHYSICS_ENTITIES,
        MEASURED_STEPS,
    );
}

#[test]
#[ignore = "manual benchmark; compares Rapier stepping with many moving colliders"]
fn step_with_many_moving_colliders() {
    let (mut backend, physics_world, _) = collider_benchmark_fixture();
    for _ in 0..WARMUP_STEPS {
        backend
            .step(physics_world, fixed_step())
            .expect("warmup step");
    }

    let started = Instant::now();
    for _ in 0..COLLIDER_MEASURED_STEPS {
        backend
            .step(physics_world, fixed_step())
            .expect("measured step");
    }
    let elapsed = started.elapsed();
    eprintln!(
        "rapier collider step: {:?} total, {:?}/step ({} moving colliders, {} steps)",
        elapsed,
        elapsed / COLLIDER_MEASURED_STEPS as u32,
        COLLIDER_BENCHMARK_ENTITIES,
        COLLIDER_MEASURED_STEPS,
    );
}

#[test]
#[ignore = "manual benchmark; compares full updates with many moving colliders"]
fn headless_step_with_many_moving_colliders() {
    let (mut backend, physics_world, mut world) = collider_benchmark_fixture();
    for _ in 0..WARMUP_STEPS {
        step_physics(&mut backend, &mut world, physics_world, fixed_step()).expect("warmup step");
    }

    let started = Instant::now();
    for _ in 0..COLLIDER_MEASURED_STEPS {
        step_physics(&mut backend, &mut world, physics_world, fixed_step()).expect("measured step");
    }
    let elapsed = started.elapsed();
    eprintln!(
        "rapier collider headless step: {:?} total, {:?}/step ({} moving colliders, {} steps)",
        elapsed,
        elapsed / COLLIDER_MEASURED_STEPS as u32,
        COLLIDER_BENCHMARK_ENTITIES,
        COLLIDER_MEASURED_STEPS,
    );
}
