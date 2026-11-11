//! Manual probe for repeated articulation metadata preparation without explicit damping.
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, MultibodyLink, PhysicsBackend, PhysicsOwnedPose, PhysicsWorldDesc, RevoluteJointDesc,
    RigidBody, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use std::time::Instant;

#[test]
#[ignore = "manual release-mode joint metadata benchmark"]
fn articulation_sync_without_passive_dynamics() {
    for count in [18, 128] {
        let mut backend = RapierBackend::new();
        let id = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        let mut world = World::new();
        let mut parent = world
            .spawn((
                RigidBody {
                    body_type: RigidBodyType::Fixed,
                    ..RigidBody::default()
                },
                Collider::sphere(0.05),
                MultibodyLink,
                Transform3::default(),
            ))
            .id();
        for _ in 0..count {
            parent = world
                .spawn((
                    RigidBody::default(),
                    Collider::sphere(0.05),
                    MultibodyLink,
                    PhysicsOwnedPose,
                    Transform3::default(),
                    RevoluteJointDesc {
                        parent,
                        axis: Vec3::Z,
                        anchor_parent_m: Vec3::ZERO,
                        anchor_child_m: Vec3::ZERO,
                        relative_rotation: Quat::IDENTITY,
                        lower_rad: None,
                        upper_rad: None,
                    },
                ))
                .id();
        }
        for _ in 0..16 {
            backend.sync_from_ecs(&mut world, id).unwrap();
        }
        let started = Instant::now();
        for _ in 0..1024 {
            backend.sync_from_ecs(&mut world, id).unwrap();
        }
        println!(
            "metadata links={count} sync_ns={}",
            started.elapsed().as_nanos() / 1024
        );
    }
}
