//! Public-API timing probe for physics-owned poses under ECS transform hierarchies.
use rne_ecs::{Parent, World};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, PhysicsBackend, PhysicsOwnedPose, PhysicsWorldDesc, RigidBody};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use std::time::Instant;

#[test]
#[ignore = "manual release-mode transform benchmark"]
fn owned_pose_sync() {
    for depth in [1, 16] {
        let mut backend = RapierBackend::new();
        let id = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        let mut world = World::new();
        let mut parent = world.spawn(Transform3::default()).id();
        for _ in 1..depth {
            parent = world
                .spawn((
                    Parent(parent),
                    Transform3::from_translation_rotation(Vec3::Y, Quat::IDENTITY),
                ))
                .id();
        }
        for index in 0..128 {
            world.spawn((
                Parent(parent),
                Transform3::from_translation_rotation(Vec3::X * index as f64, Quat::IDENTITY),
                PhysicsOwnedPose,
                RigidBody::default(),
                Collider::sphere(0.05),
            ));
        }
        for _ in 0..16 {
            backend.sync_from_ecs(&mut world, id).unwrap();
        }
        let start = Instant::now();
        for _ in 0..1024 {
            backend.sync_from_ecs(&mut world, id).unwrap();
        }
        println!(
            "owned_pose depth={depth} bodies=128 sync_ns={}",
            start.elapsed().as_nanos() / 1024
        );
    }
}
