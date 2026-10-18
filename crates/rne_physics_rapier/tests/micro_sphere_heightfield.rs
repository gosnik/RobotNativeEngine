//! Regression coverage for millimetre-scale Rapier ground contact.

use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_world::Transform3;

const DT_S: SimDuration = SimDuration::from_ticks(200_000);
const CONTACT_ERROR_M: f64 = 0.00005;
const CONTACT_FREQUENCY_HZ: f64 = 300.0;
const CONTACT_DAMPING_RATIO: f64 = 5.0;
const TOE_RADIUS_M: f64 = 0.005;
const MAX_PENETRATION_M: f64 = 0.0001;

fn settled_toe_center_y_m(heightfield: bool) -> f64 {
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc {
            gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
            solver_iterations: 12,
        })
        .expect("physics world");
    assert!(backend.set_contact_stabilization(
        physics_world,
        CONTACT_ERROR_M,
        CONTACT_FREQUENCY_HZ,
        CONTACT_DAMPING_RATIO,
    ));

    let mut world = World::new();
    let ground_shape = if heightfield {
        ColliderShape::HeightField {
            nrows: 2,
            ncols: 2,
            heights_m: vec![0.0; 4].into(),
            scale: Vec3::new(12.0, 1.0, 12.0),
        }
    } else {
        ColliderShape::Cuboid {
            half_extents_m: Vec3::new(6.0, 0.01, 6.0),
        }
    };
    let ground_y_m = if heightfield { 0.0 } else { -0.01 };
    world.spawn((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ground_shape,
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, ground_y_m, 0.0), Quat::IDENTITY),
    ));
    let toe = world
        .spawn((
            RigidBody {
                mass_kg: 11.69,
                ..RigidBody::default()
            },
            Collider::sphere(TOE_RADIUS_M),
            Transform3::from_translation_rotation(Vec3::new(0.0, 0.01, 0.0), Quat::IDENTITY),
        ))
        .id();

    for _ in 0..10_000 {
        step_physics(&mut backend, &mut world, physics_world, DT_S).expect("physics step");
    }
    world
        .get::<Transform3>(toe)
        .expect("toe pose")
        .translation
        .y
}

#[test]
fn contact_stabilization_keeps_loaded_five_mm_toe_above_flat_heightfield() {
    let center_y_m = settled_toe_center_y_m(true);
    assert!(
        (center_y_m - TOE_RADIUS_M).abs() <= MAX_PENETRATION_M,
        "heightfield resting contact error exceeded 0.1 mm: center_y_m={center_y_m}"
    );
}

#[test]
fn contact_stabilization_matches_flat_box_ground() {
    let center_y_m = settled_toe_center_y_m(false);
    assert!(
        (center_y_m - TOE_RADIUS_M).abs() <= MAX_PENETRATION_M,
        "box resting contact error exceeded 0.1 mm: center_y_m={center_y_m}"
    );
}
