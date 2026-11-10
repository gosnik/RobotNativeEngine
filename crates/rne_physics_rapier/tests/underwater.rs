//! Rapier integration regression for buoyancy-driven gliding.

use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ExternalBodyWrench, HydrodynamicCoefficients, HydrodynamicMesh, LinkKinematics,
    PhysicsBackend, PhysicsWorldDesc, RigidBody, Water,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;

fn submerged_plate() -> HydrodynamicMesh {
    let vertices = vec![
        Vec3::new(-1.0, -0.20, -0.12),
        Vec3::new(1.0, -0.20, -0.12),
        Vec3::new(1.0, 0.20, -0.12),
        Vec3::new(-1.0, 0.20, -0.12),
        Vec3::new(-1.0, -0.20, 0.12),
        Vec3::new(1.0, -0.20, 0.12),
        Vec3::new(1.0, 0.20, 0.12),
        Vec3::new(-1.0, 0.20, 0.12),
    ];
    let triangles = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [3, 7, 6],
        [3, 6, 2],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    HydrodynamicMesh::new(vertices, triangles).unwrap()
}

fn glide(mesh: &HydrodynamicMesh, pitch_rad: f64) -> Vec3 {
    let mut backend = RapierBackend::new();
    let world_id = backend.create_world(PhysicsWorldDesc::default()).unwrap();
    let mut world = World::new();
    let spawn = Vec3::new(0.0, -3.0, 0.0);
    let entity = world
        .spawn((
            Transform3::from_translation_rotation(spawn, Quat::from_rotation_z(pitch_rad)),
            RigidBody {
                mass_kg: 215.0,
                ..RigidBody::default()
            },
            Collider::cuboid(Vec3::new(1.0, 0.2, 0.12)),
        ))
        .id();
    let water = Water {
        surface_y_m: 0.0,
        density_kg_m3: 1_000.0,
        current_world_m_s: Vec3::ZERO,
        gravity_m_s2: 9.81,
    };
    let coefficients = HydrodynamicCoefficients {
        pressure_drag: 1.0,
        skin_drag: 0.02,
    };
    for _ in 0..400 {
        backend.sync_from_ecs(&mut world, world_id).unwrap();
        let pose = *world.get::<Transform3>(entity).unwrap();
        let body = *world.get::<RigidBody>(entity).unwrap();
        let wrench = mesh
            .evaluate(
                water,
                LinkKinematics {
                    transform_world: rne_math::Transform3::from_translation_rotation(
                        pose.translation,
                        pose.rotation,
                    ),
                    center_of_mass_world_m: pose.translation,
                    linear_velocity_world_m_s: body.linear_velocity_m_s,
                    angular_velocity_world_rad_s: body.angular_velocity_rad_s,
                },
                coefficients,
            )
            .unwrap();
        backend
            .apply_external_body_wrench(
                world_id,
                ExternalBodyWrench {
                    entity,
                    point_world_m: pose.translation,
                    force_world_n: wrench.force_world_n,
                    torque_world_nm: wrench.torque_world_nm,
                },
            )
            .unwrap();
        backend
            .step(world_id, SimDuration::from_ticks(5_000_000))
            .unwrap();
        backend.sync_to_ecs(&mut world, world_id).unwrap();
    }
    world.get::<Transform3>(entity).unwrap().translation - spawn
}

#[test]
fn pitched_submerged_body_converts_descent_to_horizontal_motion() {
    let mesh = submerged_plate();
    let level = glide(&mesh, 0.0);
    let nose_up = glide(&mesh, 0.35);
    let nose_down = glide(&mesh, -0.35);
    assert!(level.y < -0.5 && nose_up.y < -0.5 && nose_down.y < -0.5);
    assert!(level.x.abs() < 1.0e-5, "{level:?}");
    assert!(
        nose_up.x < -0.05 && nose_down.x > 0.05,
        "{nose_up:?} {nose_down:?}"
    );
}
