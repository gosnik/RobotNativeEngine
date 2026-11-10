//! Backend-neutral mesh hydrodynamics for articulated links.
//!
//! The model assumes a still, horizontal free surface with Y up.  It clips a
//! closed link mesh against that surface for hydrostatic displacement and
//! evaluates pressure and skin drag from the local velocity of each wetted
//! triangle.  It deliberately contains no backend handles or added-mass model.

use rne_math::{Transform3, Vec3};
use std::collections::BTreeMap;
use thiserror::Error;

/// Failure while constructing or evaluating a hydrodynamic mesh.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum HydrodynamicsError {
    /// A geometric or physical input contains a non-finite number.
    #[error("{0} must be finite")]
    NonFinite(&'static str),
    /// The mesh does not contain enough geometry to enclose a volume.
    #[error("mesh must contain at least four vertices and four triangles")]
    TooSmallMesh,
    /// A triangle references a vertex outside the vertex array.
    #[error("triangle {triangle} index {index} is outside the vertex array")]
    VertexIndexOutOfRange {
        /// Index of the triangle containing the invalid reference.
        triangle: usize,
        /// Vertex index referenced by the triangle.
        index: usize,
    },
    /// A triangle is degenerate.
    #[error("triangle {0} is degenerate")]
    DegenerateTriangle(usize),
    /// An edge is not shared by precisely two oppositely directed triangles.
    #[error("mesh is not a consistently wound closed two-manifold at edge ({0}, {1})")]
    OpenOrInconsistentEdge(usize, usize),
    /// The triangle winding does not enclose positive signed volume.
    #[error("mesh must have outward winding and positive enclosed volume")]
    NonPositiveVolume,
    /// A physical coefficient is invalid.
    #[error("{0} must be finite and non-negative")]
    NegativeOrNonFinite(&'static str),
    /// A physical magnitude must be strictly positive.
    #[error("{0} must be finite and greater than zero")]
    NonPositive(&'static str),
    /// A link transform has unsupported scale.
    #[error("link transform scale must be finite and strictly positive")]
    InvalidScale,
    /// A link transform does not contain a unit rotation quaternion.
    #[error("link transform rotation must be a finite unit quaternion")]
    InvalidRotation,
}

/// Constant water properties used by [`HydrodynamicMesh::evaluate`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Water {
    /// Horizontal free-surface height in world Y, meters.
    pub surface_y_m: f64,
    /// Fluid density in kilograms per cubic meter.
    pub density_kg_m3: f64,
    /// Fluid velocity in world coordinates, meters per second.
    pub current_world_m_s: Vec3,
    /// Positive gravitational acceleration magnitude, meters per second squared.
    pub gravity_m_s2: f64,
}

/// World-frame kinematics of the link carrying a hydrodynamic mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkKinematics {
    /// Transform from mesh-local meters to world coordinates.
    pub transform_world: Transform3,
    /// Link center of mass in world coordinates, meters.
    pub center_of_mass_world_m: Vec3,
    /// Linear velocity at the center of mass, meters per second.
    pub linear_velocity_world_m_s: Vec3,
    /// Angular velocity in world coordinates, radians per second.
    pub angular_velocity_world_rad_s: Vec3,
}

/// Dimensionless drag coefficients for a link surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HydrodynamicCoefficients {
    /// Coefficient for quadratic drag normal to each wetted triangle.
    pub pressure_drag: f64,
    /// Coefficient for quadratic drag tangent to each wetted triangle.
    pub skin_drag: f64,
}

/// Resultant hydrodynamic wrench and displaced-water diagnostics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HydrodynamicWrench {
    /// Total fluid force in world coordinates, newtons.
    pub force_world_n: Vec3,
    /// Total fluid torque about the supplied link center of mass, newton-meters.
    pub torque_world_nm: Vec3,
    /// Volume below the free surface, cubic meters.
    pub submerged_volume_m3: f64,
    /// Centroid of displaced water in world coordinates, meters.
    pub center_of_buoyancy_world_m: Vec3,
}

/// Validated, closed, outward-wound indexed mesh in link-local coordinates.
#[derive(Clone, Debug)]
pub struct HydrodynamicMesh {
    vertices: Vec<Vec3>,
    triangles: Vec<[usize; 3]>,
    full_volume_m3: f64,
    full_centroid_local_m: Vec3,
}

impl HydrodynamicMesh {
    /// Validates an outward-wound, closed indexed mesh and caches its full volume.
    pub fn new(
        vertices: Vec<Vec3>,
        triangles: Vec<[usize; 3]>,
    ) -> Result<Self, HydrodynamicsError> {
        if vertices.len() < 4 || triangles.len() < 4 {
            return Err(HydrodynamicsError::TooSmallMesh);
        }
        if vertices.iter().any(|vertex| !vertex.is_finite()) {
            return Err(HydrodynamicsError::NonFinite("mesh vertex"));
        }
        let mut edges = BTreeMap::<(usize, usize), Vec<(usize, usize)>>::new();
        let reference = vertices[0];
        let mut signed_volume_m3 = 0.0;
        let mut centroid_numerator = Vec3::ZERO;
        for (triangle_index, triangle) in triangles.iter().copied().enumerate() {
            for index in triangle {
                if index >= vertices.len() {
                    return Err(HydrodynamicsError::VertexIndexOutOfRange {
                        triangle: triangle_index,
                        index,
                    });
                }
            }
            let [ia, ib, ic] = triangle;
            let (a, b, c) = (vertices[ia], vertices[ib], vertices[ic]);
            let double_area_squared = (b - a).cross(c - a).length_squared();
            if !double_area_squared.is_finite() {
                return Err(HydrodynamicsError::NonFinite("triangle area"));
            }
            if double_area_squared == 0.0 {
                return Err(HydrodynamicsError::DegenerateTriangle(triangle_index));
            }
            for (from, to) in [(ia, ib), (ib, ic), (ic, ia)] {
                edges
                    .entry((from.min(to), from.max(to)))
                    .or_default()
                    .push((from, to));
            }
            let tetra_volume_m3 = (a - reference).dot((b - reference).cross(c - reference)) / 6.0;
            signed_volume_m3 += tetra_volume_m3;
            centroid_numerator +=
                ((a - reference) + (b - reference) + (c - reference)) * (tetra_volume_m3 / 4.0);
        }
        for ((a, b), uses) in edges {
            if uses.len() != 2 || uses[0].0 == uses[1].0 || uses[0].1 == uses[1].1 {
                return Err(HydrodynamicsError::OpenOrInconsistentEdge(a, b));
            }
        }
        if !signed_volume_m3.is_finite() {
            return Err(HydrodynamicsError::NonFinite("mesh volume"));
        }
        if signed_volume_m3 <= 0.0 {
            return Err(HydrodynamicsError::NonPositiveVolume);
        }
        let full_centroid_local_m = reference + centroid_numerator / signed_volume_m3;
        if !full_centroid_local_m.is_finite() {
            return Err(HydrodynamicsError::NonFinite("mesh centroid"));
        }
        Ok(Self {
            vertices,
            triangles,
            full_volume_m3: signed_volume_m3,
            full_centroid_local_m,
        })
    }

    /// Evaluates buoyancy and drag, returning a resultant wrench about link COM.
    pub fn evaluate(
        &self,
        water: Water,
        kinematics: LinkKinematics,
        coefficients: HydrodynamicCoefficients,
    ) -> Result<HydrodynamicWrench, HydrodynamicsError> {
        validate_inputs(water, kinematics, coefficients)?;
        let vertices: Vec<_> = self
            .vertices
            .iter()
            .map(|&point| kinematics.transform_world.transform_point(point))
            .collect();
        if vertices.iter().any(|point| !point.is_finite()) {
            return Err(HydrodynamicsError::NonFinite("transformed mesh vertex"));
        }
        let all_submerged = vertices.iter().all(|point| point.y <= water.surface_y_m);
        let all_dry = vertices.iter().all(|point| point.y >= water.surface_y_m);
        if all_dry {
            return Ok(HydrodynamicWrench {
                force_world_n: Vec3::ZERO,
                torque_world_nm: Vec3::ZERO,
                submerged_volume_m3: 0.0,
                center_of_buoyancy_world_m: Vec3::ZERO,
            });
        }
        let scale_volume = kinematics.transform_world.scale.x
            * kinematics.transform_world.scale.y
            * kinematics.transform_world.scale.z;
        let (volume_m3, center_of_buoyancy_world_m, wetted) = if all_submerged {
            (
                scale_volume * self.full_volume_m3,
                kinematics
                    .transform_world
                    .transform_point(self.full_centroid_local_m),
                self.world_triangles(&vertices),
            )
        } else {
            clipped_properties(
                &vertices,
                &self.triangles,
                water.surface_y_m,
                Vec3::new(vertices[0].x, water.surface_y_m, vertices[0].z),
            )
        };
        // Allow signed integration roundoff around zero at the surface, scaled
        // to the mesh volume. Preserve any positive displacement.
        if volume_m3 < -1.0e-12 * scale_volume * self.full_volume_m3 {
            return Err(HydrodynamicsError::NonPositiveVolume);
        }
        if volume_m3 <= 0.0 {
            return Ok(HydrodynamicWrench {
                force_world_n: Vec3::ZERO,
                torque_world_nm: Vec3::ZERO,
                submerged_volume_m3: 0.0,
                center_of_buoyancy_world_m: Vec3::ZERO,
            });
        }
        let buoyancy = Vec3::Y * (water.density_kg_m3 * water.gravity_m_s2 * volume_m3);
        let mut force = buoyancy;
        let mut torque =
            (center_of_buoyancy_world_m - kinematics.center_of_mass_world_m).cross(buoyancy);
        for triangle in wetted {
            let normal = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);
            let double_area = normal.length();
            if double_area == 0.0 {
                continue;
            }
            let area = 0.5 * double_area;
            let unit_normal = normal / double_area;
            let point = (triangle[0] + triangle[1] + triangle[2]) / 3.0;
            let relative_velocity = kinematics.linear_velocity_world_m_s
                + kinematics
                    .angular_velocity_world_rad_s
                    .cross(point - kinematics.center_of_mass_world_m)
                - water.current_world_m_s;
            let normal_speed = relative_velocity.dot(unit_normal);
            let normal_force = -unit_normal
                * (0.5
                    * water.density_kg_m3
                    * coefficients.pressure_drag
                    * area
                    * normal_speed.abs()
                    * normal_speed);
            let tangent_velocity = relative_velocity - unit_normal * normal_speed;
            let tangent_force = -tangent_velocity
                * (0.5
                    * water.density_kg_m3
                    * coefficients.skin_drag
                    * area
                    * tangent_velocity.length());
            let triangle_force = normal_force + tangent_force;
            force += triangle_force;
            torque += (point - kinematics.center_of_mass_world_m).cross(triangle_force);
        }
        let result = HydrodynamicWrench {
            force_world_n: force,
            torque_world_nm: torque,
            submerged_volume_m3: volume_m3,
            center_of_buoyancy_world_m,
        };
        if !result.force_world_n.is_finite()
            || !result.torque_world_nm.is_finite()
            || !result.submerged_volume_m3.is_finite()
            || !result.center_of_buoyancy_world_m.is_finite()
        {
            return Err(HydrodynamicsError::NonFinite("hydrodynamic result"));
        }
        Ok(result)
    }

    fn world_triangles(&self, vertices: &[Vec3]) -> Vec<[Vec3; 3]> {
        self.triangles
            .iter()
            .map(|indices| {
                [
                    vertices[indices[0]],
                    vertices[indices[1]],
                    vertices[indices[2]],
                ]
            })
            .collect()
    }
}

fn validate_inputs(
    water: Water,
    kinematics: LinkKinematics,
    coefficients: HydrodynamicCoefficients,
) -> Result<(), HydrodynamicsError> {
    if !water.surface_y_m.is_finite() {
        return Err(HydrodynamicsError::NonFinite("water surface height"));
    }
    if !water.current_world_m_s.is_finite() {
        return Err(HydrodynamicsError::NonFinite("water current"));
    }
    if !water.density_kg_m3.is_finite() || water.density_kg_m3 <= 0.0 {
        return Err(HydrodynamicsError::NonPositive("water density"));
    }
    if !water.gravity_m_s2.is_finite() || water.gravity_m_s2 <= 0.0 {
        return Err(HydrodynamicsError::NonPositive("gravity"));
    }
    if !coefficients.pressure_drag.is_finite() || coefficients.pressure_drag < 0.0 {
        return Err(HydrodynamicsError::NegativeOrNonFinite("pressure drag"));
    }
    if !coefficients.skin_drag.is_finite() || coefficients.skin_drag < 0.0 {
        return Err(HydrodynamicsError::NegativeOrNonFinite("skin drag"));
    }
    if !kinematics.transform_world.translation.is_finite()
        || !kinematics.transform_world.rotation.is_finite()
        || !kinematics.center_of_mass_world_m.is_finite()
        || !kinematics.linear_velocity_world_m_s.is_finite()
        || !kinematics.angular_velocity_world_rad_s.is_finite()
    {
        return Err(HydrodynamicsError::NonFinite("link kinematics"));
    }
    if (kinematics.transform_world.rotation.length_squared() - 1.0).abs() > 1.0e-9 {
        return Err(HydrodynamicsError::InvalidRotation);
    }
    let scale = kinematics.transform_world.scale;
    if !scale.is_finite() || scale.x <= 0.0 || scale.y <= 0.0 || scale.z <= 0.0 {
        return Err(HydrodynamicsError::InvalidScale);
    }
    Ok(())
}

fn clipped_properties(
    vertices: &[Vec3],
    triangles: &[[usize; 3]],
    surface_y_m: f64,
    reference: Vec3,
) -> (f64, Vec3, Vec<[Vec3; 3]>) {
    let mut wetted = Vec::new();
    let mut volume = 0.0;
    let mut centroid_numerator = Vec3::ZERO;
    for indices in triangles {
        let (polygon, count) = clip_triangle(
            [
                vertices[indices[0]],
                vertices[indices[1]],
                vertices[indices[2]],
            ],
            surface_y_m,
        );
        if count < 3 {
            continue;
        }
        for index in 1..count - 1 {
            let triangle = [polygon[0], polygon[index], polygon[index + 1]];
            let [a, b, c] = triangle.map(|point| point - reference);
            let tetra = a.dot(b.cross(c)) / 6.0;
            volume += tetra;
            centroid_numerator += (a + b + c) * (tetra / 4.0);
            wetted.push(triangle);
        }
    }
    let centroid = if volume > 0.0 {
        reference + centroid_numerator / volume
    } else {
        Vec3::ZERO
    };
    (volume, centroid, wetted)
}

fn clip_triangle(triangle: [Vec3; 3], surface_y_m: f64) -> ([Vec3; 4], usize) {
    let mut output = [Vec3::ZERO; 4];
    let mut count = 0;
    for index in 0..3 {
        let a = triangle[index];
        let b = triangle[(index + 1) % 3];
        let a_inside = a.y <= surface_y_m;
        let b_inside = b.y <= surface_y_m;
        if a_inside {
            output[count] = a;
            count += 1;
        }
        if a_inside != b_inside {
            let t = (surface_y_m - a.y) / (b.y - a.y);
            let point = a + (b - a) * t;
            output[count] = point;
            count += 1;
        }
    }
    (output, count)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::Quat;

    fn cube() -> HydrodynamicMesh {
        let v = vec![
            Vec3::new(-1., -1., -1.),
            Vec3::new(1., -1., -1.),
            Vec3::new(1., 1., -1.),
            Vec3::new(-1., 1., -1.),
            Vec3::new(-1., -1., 1.),
            Vec3::new(1., -1., 1.),
            Vec3::new(1., 1., 1.),
            Vec3::new(-1., 1., 1.),
        ];
        HydrodynamicMesh::new(
            v,
            vec![
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
            ],
        )
        .unwrap()
    }
    fn water() -> Water {
        Water {
            surface_y_m: 0.0,
            density_kg_m3: 1000.0,
            current_world_m_s: Vec3::ZERO,
            gravity_m_s2: 10.0,
        }
    }
    fn kin(transform: Transform3) -> LinkKinematics {
        LinkKinematics {
            transform_world: transform,
            center_of_mass_world_m: transform.translation,
            linear_velocity_world_m_s: Vec3::ZERO,
            angular_velocity_world_rad_s: Vec3::ZERO,
        }
    }
    fn coeff() -> HydrodynamicCoefficients {
        HydrodynamicCoefficients {
            pressure_drag: 1.0,
            skin_drag: 1.0,
        }
    }
    #[test]
    fn dry_and_full_buoyancy() {
        let mesh = cube();
        let dry = mesh
            .evaluate(
                water(),
                kin(Transform3::from_translation_rotation(
                    Vec3::Y * 3.,
                    Quat::IDENTITY,
                )),
                coeff(),
            )
            .unwrap();
        assert_eq!(dry.submerged_volume_m3, 0.);
        let full = mesh
            .evaluate(
                water(),
                kin(Transform3::from_translation_rotation(
                    Vec3::Y * -3.,
                    Quat::IDENTITY,
                )),
                coeff(),
            )
            .unwrap();
        assert_relative_eq!(full.submerged_volume_m3, 8.);
        assert_relative_eq!(full.force_world_n.y, 80000.);
    }
    #[test]
    fn half_submerged_cube_has_half_volume_and_center() {
        let result = cube()
            .evaluate(water(), kin(Transform3::IDENTITY), coeff())
            .unwrap();
        assert_relative_eq!(result.submerged_volume_m3, 4., epsilon = 1e-10);
        assert_relative_eq!(result.center_of_buoyancy_world_m.y, -0.5, epsilon = 1e-10);
    }

    #[test]
    fn small_closed_mesh_keeps_its_displacement() {
        let mut vertices = cube().vertices;
        for vertex in &mut vertices {
            *vertex *= 0.0005;
        }
        let triangles = cube().triangles;
        let mesh = HydrodynamicMesh::new(vertices.clone(), triangles.clone()).unwrap();
        let result = mesh
            .evaluate(water(), kin(Transform3::IDENTITY), coeff())
            .unwrap();
        assert_relative_eq!(result.submerged_volume_m3, 5.0e-10, epsilon = 1.0e-20);
        vertices[1] = vertices[0];
        assert_eq!(
            HydrodynamicMesh::new(vertices, triangles).unwrap_err(),
            HydrodynamicsError::DegenerateTriangle(0)
        );
    }
    #[test]
    fn current_drag_opposes_relative_motion() {
        let result = cube()
            .evaluate(
                Water {
                    current_world_m_s: -Vec3::X,
                    ..water()
                },
                LinkKinematics {
                    ..kin(Transform3::from_translation_rotation(
                        Vec3::Y * -3.,
                        Quat::IDENTITY,
                    ))
                },
                coeff(),
            )
            .unwrap();
        assert!(result.force_world_n.x < 0.);
    }
    #[test]
    fn rotational_drag_dissipates_energy() {
        let angular_velocity = Vec3::Y;
        let result = cube()
            .evaluate(
                water(),
                LinkKinematics {
                    angular_velocity_world_rad_s: angular_velocity,
                    ..kin(Transform3::from_translation_rotation(
                        Vec3::Y * -3.,
                        Quat::IDENTITY,
                    ))
                },
                coeff(),
            )
            .unwrap();
        assert!(result.torque_world_nm.dot(angular_velocity) < 0.);
    }
    #[test]
    fn tilted_offset_body_has_finite_partial_wrench() {
        let transform = Transform3::from_translation_rotation(
            Vec3::new(0.4, 0., -0.2),
            Quat::from_rotation_z(0.35),
        );
        let result = cube().evaluate(water(), kin(transform), coeff()).unwrap();
        assert!(result.submerged_volume_m3 > 0. && result.submerged_volume_m3 < 8.);
        assert!(result.force_world_n.is_finite() && result.torque_world_nm.is_finite());
    }
}
