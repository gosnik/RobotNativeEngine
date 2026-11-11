//! Contact-heavy public-API benchmark for comparing scalar and SIMD solvers.

use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    hash_physics_state_v2, Collider, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use std::time::Instant;

fn dt() -> SimDuration {
    SimDuration::from_hertz(Hertz::new(60.0))
}

#[test]
#[ignore = "release-mode contact performance benchmark"]
fn sliding_contacts() {
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
            solver_iterations: 4,
        })
        .unwrap();
    let mut world = World::new();

    world.spawn((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        {
            let mut c = Collider::cuboid(Vec3::new(20.0, 0.05, 20.0));
            c.material.friction = 0.0;
            c
        },
        Transform3::from_translation_rotation(Vec3::new(2.25, -0.05, 2.25), Quat::IDENTITY),
    ));
    for row in 0..16 {
        for col in 0..16 {
            world.spawn((
                RigidBody {
                    linear_velocity_m_s: Vec3::X,
                    ..RigidBody::default()
                },
                {
                    let mut c = Collider::cuboid(Vec3::splat(0.05));
                    c.material.friction = 0.0;
                    c
                },
                Transform3::from_translation_rotation(
                    Vec3::new(col as f64 * 0.3, 0.05, row as f64 * 0.3),
                    Quat::IDENTITY,
                ),
            ));
        }
    }
    backend.sync_from_ecs(&mut world, physics_world).unwrap();
    for _ in 0..16 {
        backend.step(physics_world, dt()).unwrap();
    }
    assert_eq!(backend.contacts(physics_world).unwrap().len(), 256);
    let started = Instant::now();
    for _ in 0..128 {
        backend.step(physics_world, dt()).unwrap();
    }
    let elapsed = started.elapsed();
    println!("physics_step_ns={}", elapsed.as_nanos() / 128);
    backend.sync_to_ecs(&mut world, physics_world).unwrap();
    assert_eq!(backend.contacts(physics_world).unwrap().len(), 256);
    println!(
        "sliding_contacts hash={:016x} contacts={}",
        hash_physics_state_v2(&world),
        backend.contacts(physics_world).unwrap().len()
    );
}
