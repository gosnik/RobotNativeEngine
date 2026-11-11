//! Rapier regressions for selectable ground types (normal, rock, sand, mud).

use rne_core::SimDuration;
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::{
    Collider, GroundMaterial, GroundSurface, GroundType, PhysicsBackend, PhysicsWorldDesc,
    PhysicsWorldId, RigidBody, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;

const DT_NS: u64 = 1_000_000;
const TOE_RADIUS_M: f64 = 0.015;
const TOE_MASS_KG: f64 = 1.5;

struct Scene {
    backend: RapierBackend,
    world: World,
    world_id: PhysicsWorldId,
    ground: Entity,
    body: Entity,
}

impl Scene {
    fn new(ground: Option<GroundType>, body: (Collider, f64, Vec3, Vec3)) -> Self {
        let mut backend = RapierBackend::new();
        let world_id = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        let mut world = World::new();
        let ground_entity = world
            .spawn((
                Transform3::from_translation_rotation(
                    Vec3::new(0.0, -0.5, 0.0),
                    rne_math::Quat::IDENTITY,
                ),
                RigidBody {
                    body_type: RigidBodyType::Fixed,
                    ..RigidBody::default()
                },
                Collider::cuboid(Vec3::new(5.0, 0.5, 5.0)),
            ))
            .id();
        if let Some(kind) = ground {
            world
                .entity_mut(ground_entity)
                .insert(GroundSurface::new(kind));
        }
        let (collider, mass_kg, position, velocity) = body;
        let body = world
            .spawn((
                Transform3::from_translation_rotation(position, rne_math::Quat::IDENTITY),
                RigidBody {
                    mass_kg,
                    linear_velocity_m_s: velocity,
                    ..RigidBody::default()
                },
                collider,
            ))
            .id();
        Self {
            backend,
            world,
            world_id,
            ground: ground_entity,
            body,
        }
    }

    fn toe(ground: Option<GroundType>, drop_speed_m_s: f64) -> Self {
        Self::new(
            ground,
            (
                Collider::sphere(TOE_RADIUS_M),
                TOE_MASS_KG,
                Vec3::new(0.0, TOE_RADIUS_M + 0.002, 0.0),
                Vec3::new(0.0, -drop_speed_m_s, 0.0),
            ),
        )
    }

    fn step(&mut self) {
        self.backend
            .sync_from_ecs(&mut self.world, self.world_id)
            .unwrap();
        self.backend
            .step(self.world_id, SimDuration::from_ticks(DT_NS))
            .unwrap();
        self.backend
            .sync_to_ecs(&mut self.world, self.world_id)
            .unwrap();
    }

    fn position(&self) -> Vec3 {
        self.world.get::<Transform3>(self.body).unwrap().translation
    }

    fn ground_force_n(&self) -> f64 {
        self.backend
            .ground_contacts(self.world_id)
            .unwrap()
            .iter()
            .map(|contact| contact.normal_force_n)
            .sum()
    }
}

struct DropTrace {
    peak_force_n: f64,
    sinkage_at_m: Vec<f64>,
}

/// Drops the toe at 0.5 m/s and records peak force and sinkage over time.
fn drop_toe(ground: Option<GroundType>, steps: usize) -> DropTrace {
    drop_toe_at(ground, 0.5, steps)
}

fn drop_toe_at(ground: Option<GroundType>, speed_m_s: f64, steps: usize) -> DropTrace {
    let mut scene = Scene::toe(ground, speed_m_s);
    let mut peak_force_n: f64 = 0.0;
    let mut sinkage_at_m = Vec::with_capacity(steps);
    for _ in 0..steps {
        scene.step();
        peak_force_n = peak_force_n.max(scene.ground_force_n());
        sinkage_at_m.push((TOE_RADIUS_M - scene.position().y).max(0.0));
    }
    DropTrace {
        peak_force_n,
        sinkage_at_m,
    }
}

fn weight_n() -> f64 {
    TOE_MASS_KG * 9.81
}

#[test]
fn ground_type_names_round_trip() {
    for kind in GroundType::ALL {
        assert_eq!(GroundType::from_name(kind.name()), Some(kind));
        assert!(GroundMaterial::preset(kind).is_valid());
    }
    assert_eq!(GroundType::from_name("lava"), None);
}

#[test]
fn rigid_ground_supports_the_toe_at_the_surface() {
    for kind in [GroundType::Normal, GroundType::Rock] {
        let trace = drop_toe(Some(kind), 800);
        let settled = *trace.sinkage_at_m.last().unwrap();
        assert!(settled < 0.002, "{kind:?} sank {settled} m");
    }
}

#[test]
fn normal_ground_matches_a_plain_collider() {
    let mut plain = Scene::toe(None, 0.5);
    let mut normal = Scene::toe(Some(GroundType::Normal), 0.5);
    for _ in 0..300 {
        plain.step();
        normal.step();
        assert_eq!(plain.position(), normal.position());
    }
}

#[test]
fn sand_yields_until_it_bears_the_load_and_then_holds() {
    let soil = GroundMaterial::preset(GroundType::Sand)
        .deformation
        .unwrap();
    let pad_m = TOE_RADIUS_M.max(soil.min_contact_radius_m);
    let equilibrium_m = soil.equilibrium_sinkage_m(weight_n(), pad_m);
    // Released from rest on the surface: plastic ground cannot spring back,
    // so the toe overshoots the static equilibrium, then stays put.
    let trace = drop_toe_at(Some(GroundType::Sand), 0.0, 1500);
    let settled = *trace.sinkage_at_m.last().unwrap();
    assert!(
        settled > equilibrium_m && settled < 2.5 * equilibrium_m,
        "settled {settled} m, Bekker equilibrium {equilibrium_m} m"
    );
    assert!(soil.static_bearing_n(settled, pad_m) > weight_n());
    // Sand does not creep: the last half second moves less than a millimetre
    // (Rapier's penetration slop).
    let half_way = trace.sinkage_at_m[1000];
    assert!(settled - half_way < 1.0e-3, "{half_way} -> {settled}");
}

#[test]
fn faster_touchdown_sinks_deeper_into_sand() {
    let gentle = *drop_toe_at(Some(GroundType::Sand), 0.0, 800)
        .sinkage_at_m
        .last()
        .unwrap();
    let fast = *drop_toe_at(Some(GroundType::Sand), 0.5, 800)
        .sinkage_at_m
        .last()
        .unwrap();
    assert!(fast > gentle + 0.002, "gentle {gentle} m, fast {fast} m");
}

#[test]
fn mud_keeps_creeping_under_sustained_load() {
    let trace = drop_toe(Some(GroundType::Mud), 1500);
    let early = trace.sinkage_at_m[300];
    let late = *trace.sinkage_at_m.last().unwrap();
    assert!(early > 0.002, "mud should sink on impact: {early}");
    assert!(late > early + 0.01, "mud should creep: {early} -> {late}");
    // Creep is smooth: consecutive steps never jump by more than 0.2 mm.
    for pair in trace.sinkage_at_m[300..].windows(2) {
        assert!((pair[1] - pair[0]).abs() < 2.0e-4, "{pair:?}");
    }
}

#[test]
fn soft_ground_softens_the_impact() {
    let rock = drop_toe(Some(GroundType::Rock), 200).peak_force_n;
    let normal = drop_toe(Some(GroundType::Normal), 200).peak_force_n;
    let sand = drop_toe(Some(GroundType::Sand), 200).peak_force_n;
    let mud = drop_toe(Some(GroundType::Mud), 200).peak_force_n;
    // A rigid stop of 0.5 m/s in one 1 ms step is ~50 times body weight.
    assert!(rock > 10.0 * weight_n() && normal > 10.0 * weight_n());
    assert!(sand < 0.5 * normal, "sand {sand} N vs normal {normal} N");
    assert!(mud < 0.5 * normal, "mud {mud} N vs normal {normal} N");
}

/// Slides a block at 1 m/s and returns how far it travels in one second.
fn slide_distance_m(ground: GroundType) -> f64 {
    slide_distance_on_m(GroundSurface::new(ground), 0.0)
}

/// Slides a block from `x_m` at 1 m/s over a 10 m ground slab with `surface`
/// and returns how far it travels in one second.
fn slide_distance_on_m(surface: GroundSurface, x_m: f64) -> f64 {
    // Flat enough that the braking torque cannot rock it onto its edge.
    let half = Vec3::new(0.05, 0.005, 0.05);
    let mut scene = Scene::new(
        None,
        (
            Collider::cuboid(half),
            1.0,
            Vec3::new(x_m, half.y, 0.0),
            Vec3::ZERO,
        ),
    );
    scene.world.entity_mut(scene.ground).insert(surface);
    // Let the block settle (and sink into soft ground) before it slides.
    for _ in 0..500 {
        scene.step();
    }
    let start = scene.position();
    scene
        .world
        .get_mut::<RigidBody>(scene.body)
        .unwrap()
        .linear_velocity_m_s = Vec3::new(1.0, 0.0, 0.0);
    for _ in 0..1000 {
        scene.step();
    }
    scene.position().x - start.x
}

#[test]
fn friction_orders_rock_normal_sand_mud() {
    let rock = slide_distance_m(GroundType::Rock);
    let normal = slide_distance_m(GroundType::Normal);
    let mud = slide_distance_m(GroundType::Mud);
    assert!(rock < normal, "rock {rock} m, normal {normal} m");
    assert!(normal < mud, "normal {normal} m, mud {mud} m");
}

#[test]
fn ground_contacts_report_type_sinkage_and_force() {
    let mut scene = Scene::toe(Some(GroundType::Sand), 0.0);
    for _ in 0..1000 {
        scene.step();
    }
    let contacts = scene.backend.ground_contacts(scene.world_id).unwrap();
    assert!(!contacts.is_empty());
    let contact = contacts[0];
    assert_eq!(contact.ground_type, GroundType::Sand);
    assert_eq!(contact.entity, scene.body);
    assert_eq!(contact.ground_entity, scene.ground);
    assert!(contact.sinkage_m > 0.002);
    assert!(contact.ground_normal_world.y > 0.99);
    let total: f64 = contacts.iter().map(|c| c.normal_force_n).sum();
    // Same force convention as resting on rigid ground.
    let mut rigid = Scene::toe(Some(GroundType::Normal), 0.0);
    for _ in 0..1000 {
        rigid.step();
    }
    let rigid_total = rigid.ground_force_n();
    assert!(
        (total - rigid_total).abs() < 0.02 * rigid_total,
        "{total} N on sand vs {rigid_total} N on rigid ground"
    );
    assert!(contact.bearing_capacity_n.is_finite());
    assert!(contact.bearing_capacity_n >= contact.normal_force_n * 0.95);
    assert!(!contact.yielding);
    let rigid_contact = rigid.backend.ground_contacts(rigid.world_id).unwrap()[0];
    assert_eq!(rigid_contact.ground_type, GroundType::Normal);
    assert!(rigid_contact.bearing_capacity_n.is_infinite());
}

#[test]
fn removing_the_surface_restores_rigid_ground() {
    let mut scene = Scene::toe(Some(GroundType::Sand), 0.0);
    for _ in 0..200 {
        scene.step();
    }
    let sunk_m = TOE_RADIUS_M - scene.position().y;
    assert!(sunk_m > 0.02, "{sunk_m}");
    scene
        .world
        .entity_mut(scene.ground)
        .remove::<GroundSurface>();
    scene.step();
    assert!(scene
        .backend
        .ground_contacts(scene.world_id)
        .unwrap()
        .is_empty());
    // The displaced sand is gone; rigid ground pushes the toe back out.
    for _ in 0..1000 {
        scene.step();
    }
    let depth_m = TOE_RADIUS_M - scene.position().y;
    assert!(depth_m < 0.005, "toe still {depth_m} m deep");
}

#[test]
fn articulated_toe_sinks_like_a_free_body() {
    // Robot feet are multibody links, which use Rapier's generic contact path.
    let mut free = Scene::toe(Some(GroundType::Sand), 0.0);
    let mut articulated = Scene::toe(Some(GroundType::Sand), 0.0);
    articulated
        .world
        .entity_mut(articulated.body)
        .insert(rne_physics::MultibodyLink);
    for _ in 0..800 {
        free.step();
        articulated.step();
    }
    let free_m = TOE_RADIUS_M - free.position().y;
    let articulated_m = TOE_RADIUS_M - articulated.position().y;
    assert!(free_m > 0.015, "{free_m}");
    assert!(
        (articulated_m - free_m).abs() < 0.2 * free_m,
        "articulated {articulated_m} m vs free {free_m} m"
    );
}

#[test]
fn toe_sinks_into_a_height_field_without_falling_through() {
    let mut scene = Scene::toe(Some(GroundType::Mud), 0.0);
    let flat = rne_physics::ColliderShape::HeightField {
        nrows: 11,
        ncols: 11,
        heights_m: vec![0.0; 121].into(),
        scale: Vec3::new(2.0, 1.0, 2.0),
    };
    scene.world.entity_mut(scene.ground).insert((
        Transform3::from_translation_rotation(Vec3::ZERO, rne_math::Quat::IDENTITY),
        Collider {
            shape: flat,
            ..Collider::default()
        },
    ));
    // Rest the toe over a triangle diagonal, where several faces touch it.
    scene
        .world
        .get_mut::<Transform3>(scene.body)
        .unwrap()
        .translation = Vec3::new(0.03, TOE_RADIUS_M, 0.04);
    let mut previous = 0.0;
    for step in 0..2000 {
        scene.step();
        let sinkage = TOE_RADIUS_M - scene.position().y;
        assert!(sinkage < 0.13, "fell through at step {step}: {sinkage} m");
        assert!(sinkage - previous < 2.0e-3, "jump at step {step}");
        previous = sinkage;
    }
    // Mud creeps down to the firm layer and stops there.
    let max_sinkage = GroundMaterial::preset(GroundType::Mud)
        .deformation
        .unwrap()
        .max_sinkage_m;
    assert!(
        previous > 0.03 && previous < max_sinkage + 0.003,
        "{previous}"
    );
}

#[test]
fn raycasts_hit_the_undisturbed_soil_surface() {
    let mut scene = Scene::toe(Some(GroundType::Sand), 0.0);
    scene.step();
    let hits = scene
        .backend
        .raycast(
            scene.world_id,
            rne_physics::RaycastQuery::downward(Vec3::new(1.0, 1.0, 1.0), 5.0),
        )
        .unwrap();
    let ground_hit = hits.iter().find(|hit| hit.entity == scene.ground).unwrap();
    assert!(
        ground_hit.point_m.y.abs() < 1.0e-4,
        "{:?}",
        ground_hit.point_m
    );
}

/// Rests toes on a flat 2 m × 2 m height field with `surface` and returns,
/// after `steps`, each toe's sinkage and the ground type reported under it.
fn toes_on_height_field(
    surface: GroundSurface,
    toes_xz: &[(f64, f64)],
    steps: usize,
) -> Vec<(f64, Option<GroundType>)> {
    let mut backend = RapierBackend::new();
    let world_id = backend.create_world(PhysicsWorldDesc::default()).unwrap();
    let mut world = World::new();
    world.spawn((
        Transform3::from_translation_rotation(Vec3::ZERO, rne_math::Quat::IDENTITY),
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: rne_physics::ColliderShape::HeightField {
                nrows: 5,
                ncols: 5,
                heights_m: vec![0.0; 25].into(),
                scale: Vec3::new(2.0, 1.0, 2.0),
            },
            ..Collider::default()
        },
        surface,
    ));
    let toes: Vec<Entity> = toes_xz
        .iter()
        .map(|&(x, z)| {
            world
                .spawn((
                    Transform3::from_translation_rotation(
                        Vec3::new(x, TOE_RADIUS_M, z),
                        rne_math::Quat::IDENTITY,
                    ),
                    RigidBody {
                        mass_kg: TOE_MASS_KG,
                        ..RigidBody::default()
                    },
                    Collider::sphere(TOE_RADIUS_M),
                ))
                .id()
        })
        .collect();
    for _ in 0..steps {
        backend.sync_from_ecs(&mut world, world_id).unwrap();
        backend
            .step(world_id, SimDuration::from_ticks(DT_NS))
            .unwrap();
        backend.sync_to_ecs(&mut world, world_id).unwrap();
    }
    let samples = backend.ground_contacts(world_id).unwrap();
    toes.iter()
        .map(|&toe| {
            let y = world.get::<Transform3>(toe).unwrap().translation.y;
            let kind = samples
                .iter()
                .find(|sample| sample.entity == toe)
                .map(|sample| sample.ground_type);
            (TOE_RADIUS_M - y, kind)
        })
        .collect()
}

/// 5 × 5 map over the 2 m field: rock for x ≤ −0.5, normal at x = 0, sand
/// for x ≥ 0.5 except mud along the +Z edge.
fn mixed_map() -> rne_physics::GroundTypeMap {
    use GroundType::*;
    let mut types = Vec::new();
    for row in 0..5 {
        for column in 0..5 {
            types.push(match column {
                0 | 1 => Rock,
                2 => Normal,
                _ if row == 4 => Mud,
                _ => Sand,
            });
        }
    }
    rne_physics::GroundTypeMap::from_ground_types(5, 5, &types).unwrap()
}

#[test]
fn mixed_ground_map_gives_each_toe_its_own_material() {
    let toes = [(-0.6, 0.0), (0.0, 0.0), (0.6, 0.0), (0.6, 0.9)];
    let mixed = toes_on_height_field(GroundSurface::mapped(mixed_map()), &toes, 1500);
    let kinds: Vec<_> = mixed.iter().map(|(_, kind)| *kind).collect();
    assert_eq!(
        kinds,
        [
            Some(GroundType::Rock),
            Some(GroundType::Normal),
            Some(GroundType::Sand),
            Some(GroundType::Mud)
        ]
    );
    // Rigid patches stay rigid although the collider is lowered for the mud.
    assert!(mixed[0].0 < 0.002, "rock {}", mixed[0].0);
    assert!(mixed[1].0 < 0.002, "normal {}", mixed[1].0);
    // Soft patches behave as they do on uniform ground of that kind.
    for (index, kind) in [(2, GroundType::Sand), (3, GroundType::Mud)] {
        let uniform = toes_on_height_field(GroundSurface::new(kind), &[toes[index]], 1500)[0].0;
        assert!(uniform > 0.004, "{kind:?} uniform {uniform}");
        assert!(
            (mixed[index].0 - uniform).abs() < 1.0e-3,
            "{kind:?}: mixed {} m, uniform {uniform} m",
            mixed[index].0
        );
    }
}

#[test]
fn rigid_only_map_leaves_the_collider_in_place() {
    use GroundType::*;
    let map = rne_physics::GroundTypeMap::from_ground_types(2, 1, &[Rock, Normal]).unwrap();
    let result = toes_on_height_field(GroundSurface::mapped(map), &[(-0.9, 0.0), (0.9, 0.0)], 600);
    assert_eq!(result[0].1, Some(Rock));
    assert_eq!(result[1].1, Some(Normal));
    assert!(
        result.iter().all(|(sinkage, _)| *sinkage < 0.002),
        "{result:?}"
    );
}

#[test]
fn mixed_ground_map_sets_friction_per_patch() {
    use GroundType::*;
    // Over the 10 m slab: normal for x < 0, rock for x > 0.
    let map = rne_physics::GroundTypeMap::from_ground_types(2, 1, &[Normal, Rock]).unwrap();
    let on_normal = slide_distance_on_m(GroundSurface::mapped(map.clone()), -3.0);
    let on_rock = slide_distance_on_m(GroundSurface::mapped(map), 1.0);
    let uniform_normal = slide_distance_m(Normal);
    let uniform_rock = slide_distance_m(Rock);
    assert!(
        (on_normal - uniform_normal).abs() < 1.0e-3,
        "{on_normal} vs {uniform_normal}"
    );
    assert!(
        (on_rock - uniform_rock).abs() < 1.0e-3,
        "{on_rock} vs {uniform_rock}"
    );
    assert!(
        on_rock < on_normal - 0.02,
        "rock {on_rock} m, normal {on_normal} m"
    );
}

/// Runs one body on a 2 m × 2 m height field tilted along X by `slope_deg`,
/// with `surface` (or a plain collider), and returns its pose each step.
fn pose_trace_on_height_field(
    surface: Option<GroundSurface>,
    body: Collider,
    start: Vec3,
    velocity: Vec3,
    slope_deg: f64,
    steps: usize,
) -> Vec<(Vec3, rne_math::Quat)> {
    let mut backend = RapierBackend::new();
    let world_id = backend.create_world(PhysicsWorldDesc::default()).unwrap();
    let mut world = World::new();
    let n = 21;
    let grade = slope_deg.to_radians().tan();
    let heights: Vec<f64> = (0..n * n)
        .map(|i| (i % n) as f64 / (n - 1) as f64 * 2.0 * grade)
        .collect();
    let ground = world
        .spawn((
            Transform3::from_translation_rotation(Vec3::ZERO, rne_math::Quat::IDENTITY),
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: rne_physics::ColliderShape::HeightField {
                    nrows: n as u32,
                    ncols: n as u32,
                    heights_m: heights.into(),
                    scale: Vec3::new(2.0, 1.0, 2.0),
                },
                ..Collider::default()
            },
        ))
        .id();
    if let Some(surface) = surface {
        world.entity_mut(ground).insert(surface);
    }
    let body = world
        .spawn((
            Transform3::from_translation_rotation(start, rne_math::Quat::IDENTITY),
            RigidBody {
                mass_kg: 1.0,
                linear_velocity_m_s: velocity,
                ..RigidBody::default()
            },
            body,
        ))
        .id();
    (0..steps)
        .map(|_| {
            backend.sync_from_ecs(&mut world, world_id).unwrap();
            backend
                .step(world_id, SimDuration::from_ticks(DT_NS))
                .unwrap();
            backend.sync_to_ecs(&mut world, world_id).unwrap();
            let pose = world.get::<Transform3>(body).unwrap();
            (pose.translation, pose.rotation)
        })
        .collect()
}

/// Normal ground everywhere except one mud sample in the far corner, so the
/// collider is lowered and skinned for soil.
fn normal_map_with_distant_mud() -> GroundSurface {
    let mut types = vec![GroundType::Normal; 25];
    types[24] = GroundType::Mud;
    GroundSurface::mapped(rne_physics::GroundTypeMap::from_ground_types(5, 5, &types).unwrap())
}

#[test]
fn normal_cells_of_a_lowered_map_behave_like_plain_terrain() {
    // The mud sample lowers and skins the whole collider. Normal cells must
    // still support bodies at the authored surface, with the friction and
    // lever arms of plain terrain.
    let slope_deg: f64 = 20.0;
    let grade = slope_deg.to_radians().tan();
    let cases = [
        // A small toe rolling across a 20° slope and over triangle edges.
        (
            Collider::sphere(0.01),
            -0.3,
            Vec3::new(0.3, 0.0, 0.2),
            slope_deg,
            0.012,
            180.0,
        ),
        // A block resting on the slope.
        (
            Collider::cuboid(Vec3::new(0.05, 0.02, 0.05)),
            -0.3,
            Vec3::ZERO,
            slope_deg,
            0.004,
            4.0,
        ),
    ];
    for (body, x, velocity, slope, max_offset_m, max_angle_deg) in cases {
        let grade_here = slope.to_radians().tan();
        let start = Vec3::new(x, (x + 1.0) * grade_here + 0.021, 0.0);
        let plain = pose_trace_on_height_field(None, body.clone(), start, velocity, slope, 600);
        let mapped = pose_trace_on_height_field(
            Some(normal_map_with_distant_mud()),
            body.clone(),
            start,
            velocity,
            slope,
            600,
        );
        for (step, ((p0, q0), (p1, q1))) in plain.iter().zip(&mapped).enumerate() {
            let offset = (*p0 - *p1).length();
            let angle = q0.angle_between(*q1).to_degrees();
            assert!(
                offset < max_offset_m && angle < max_angle_deg,
                "{body:?} step {step}: {offset} m, {angle} deg"
            );
        }
        if body.shape == Collider::sphere(0.01).shape {
            // Rests on the authored slope (centre r / cos θ above it) and never
            // sinks into the skin, unlike the edge-contact bug that let it
            // drop 3 cm.
            let rest_m = 0.01 / slope_deg.to_radians().cos();
            for (step, (p, _)) in mapped.iter().enumerate().skip(100) {
                let above = p.y - (p.x + 1.0) * grade;
                assert!((above - rest_m).abs() < 1.5e-3, "step {step}: {above} m");
            }
        }
    }
    // A block braking from 1 m/s on flat normal cells slides v²/(2μg) with the
    // combined μ = 0.5 (plain height fields can instead catch a triangle edge).
    let body = Collider::cuboid(Vec3::new(0.05, 0.02, 0.05));
    let start = Vec3::new(-0.5, 0.021, 0.0);
    let trace = pose_trace_on_height_field(
        Some(normal_map_with_distant_mud()),
        body,
        start,
        Vec3::new(1.0, 0.0, 0.0),
        0.0,
        600,
    );
    let (end, rotation) = *trace.last().unwrap();
    let expected_m = 1.0 / (2.0 * 0.5 * 9.81);
    let slid_m = end.x - start.x;
    assert!(
        (slid_m - expected_m).abs() < 0.08 * expected_m,
        "{slid_m} m"
    );
    assert!(rotation.angle_between(rne_math::Quat::IDENTITY) < 0.01);
}

#[test]
fn encoded_map_sets_softness_and_friction_per_pixel() {
    use rne_physics::{GroundMapEncoding, GroundTypeMap};
    let encoding = GroundMapEncoding::default();
    let sand = encoding.encode(&GroundMaterial::preset(GroundType::Sand));
    let soft_sand = [sand[0], sand[1], 0xC0];
    let map = GroundTypeMap::from_encoded_pixels(2, 1, &[sand, soft_sand].concat(), 3, &encoding)
        .unwrap();
    let toes = toes_on_height_field(GroundSurface::mapped(map), &[(-0.9, 0.0), (0.9, 0.0)], 1500);
    assert_eq!(toes[0].1, Some(GroundType::Sand));
    let uniform = toes_on_height_field(GroundSurface::new(GroundType::Sand), &[(-0.9, 0.0)], 1500);
    assert!(
        (toes[0].0 - uniform[0].0).abs() < 1.0e-3,
        "{toes:?} vs {uniform:?}"
    );
    // 4× softer moduli sink a good deal deeper.
    assert!(toes[1].0 > 1.5 * toes[0].0, "{toes:?}");

    // G = 115 is friction 0.902 on normal ground: it slides like rock.
    let grippy = GroundTypeMap::from_encoded_pixels(1, 1, &[0, 115, 0], 3, &encoding).unwrap();
    let on_grippy = slide_distance_on_m(GroundSurface::mapped(grippy), 0.0);
    let on_rock = slide_distance_m(GroundType::Rock);
    assert!(
        (on_grippy - on_rock).abs() < 0.003,
        "{on_grippy} vs {on_rock}"
    );
}
