//! Public-API benchmark for direct-effort bookkeeping at robot and fleet scales.
use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    hash_physics_state_v2, Collider, JointActuation, PhysicsBackend, PhysicsWorldDesc,
    RevoluteJointDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_world::Transform3;
use std::time::{Duration, Instant};

#[test]
#[ignore = "manual release-mode effort benchmark"]
fn direct_effort_bookkeeping() {
    for count in [18, 256] {
        let mut backend = RapierBackend::new();
        let id = backend
            .create_world(PhysicsWorldDesc {
                gravity_m_s2: Vec3::ZERO,
                solver_iterations: 4,
            })
            .unwrap();
        let mut world = World::new();
        for i in 0..count {
            let position = Vec3::X * (i as f64 * 3.0);
            let parent = world
                .spawn((
                    RigidBody {
                        body_type: RigidBodyType::Fixed,
                        ..RigidBody::default()
                    },
                    Collider::sphere(0.05),
                    Transform3::from_translation_rotation(position, Quat::IDENTITY),
                ))
                .id();
            world.spawn((
                RigidBody::default(),
                Collider::sphere(0.05),
                Transform3::from_translation_rotation(position - Vec3::Y, Quat::IDENTITY),
                RevoluteJointDesc {
                    parent,
                    axis: Vec3::Z,
                    anchor_parent_m: Vec3::ZERO,
                    anchor_child_m: Vec3::Y,
                    relative_rotation: Quat::IDENTITY,
                    lower_rad: None,
                    upper_rad: None,
                },
                JointActuation::RevoluteEffort {
                    effort_nm: 0.1,
                    max_effort_nm: 1.0,
                },
            ));
        }
        let dt = SimDuration::from_hertz(Hertz::new(1000.0));
        for _ in 0..16 {
            step_physics(&mut backend, &mut world, id, dt).unwrap();
        }
        let mut sync = Duration::ZERO;
        let started = Instant::now();
        for _ in 0..256 {
            let s = Instant::now();
            backend.sync_from_ecs(&mut world, id).unwrap();
            sync += s.elapsed();
            backend.step(id, dt).unwrap();
            backend.sync_to_ecs(&mut world, id).unwrap();
        }
        let full = started.elapsed();
        println!(
            "effort count={count} sync_ns={} full_ns={} hash={:016x}",
            sync.as_nanos() / 256,
            full.as_nanos() / 256,
            hash_physics_state_v2(&world)
        );
    }
}
