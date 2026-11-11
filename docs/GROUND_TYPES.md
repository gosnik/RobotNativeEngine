# Ground types

Terrain colliders can be marked as one of four kinds of ground with the
`GroundSurface` component (`rne_physics`). The Rapier backend gives each kind
its own contact behaviour, so a legged robot's feet land, sink and slip
differently on each. Backends without ground-surface support treat the collider
as plain rigid ground with its own `PhysicsMaterial`.

```rust
use rne_physics::{GroundSurface, GroundType};

world.entity_mut(terrain).insert(GroundSurface::new(GroundType::Sand));
let kind = GroundType::from_name("mud"); // "normal", "rock", "sand", "mud"
```

`GroundMaterial::preset(kind)` returns the nominal material, and every field can
be overridden before inserting the component. The presets are simulation
assumptions taken from textbook terramechanics, not measurements of a site.

| Kind   | Support                                   | Friction (with a 1.0 rubber toe) | Restitution |
|--------|-------------------------------------------|----------------------------------|-------------|
| normal | rigid                                     | 0.5 surface, mean with the toe   | 0           |
| rock   | rigid                                     | 0.9 surface, mean with the toe   | 0.05        |
| sand   | yields plastically, little rate effect    | capped at 0.53 (tan 28°) + bulldozing | 0      |
| mud    | yields, strongly viscous, creeps under load | capped at 0.25 + bulldozing    | 0           |

`normal` behaves exactly like a plain collider with the default material, so
existing scenes are unchanged.

## Mixed terrain from a ground map

One collider can mix kinds with a `GroundTypeMap`, a grid of materials laid over
the collider's horizontal footprint. Paint it as an image the same size as the
terrain's heightmap, so each height sample gets one ground type:

```rust
use rne_physics::{GroundSurface, GroundTypeMap};

let map = GroundTypeMap::from_png(include_bytes!("terrain-ground.png"),
                                  &GroundTypeMap::default_legend())?;
world.entity_mut(terrain).insert(GroundSurface::mapped(map));
```

Default colour key (`GroundType::map_color_rgb`); each pixel takes the nearest
colour, so anti-aliased brush edges are fine:

| Kind   | Colour              |
|--------|---------------------|
| normal | green `#288C28`     |
| rock   | grey `#808080`      |
| sand   | yellow `#E6C850`    |
| mud    | brown `#643C14`     |

- Layout matches `ColliderShape::HeightField`: image columns run along local +X,
  image rows along local +Z, and samples sit on the grid vertices. Pixel
  `(0, 0)` is the −X/−Z corner and the last pixel the +X/+Z corner. A contact
  takes the material of the nearest pixel.
- Maps of any resolution are stretched over the footprint (the height field's
  `scale.x` × `scale.z`, or another shape's local bounding box). Points beyond
  the edge take the edge pixel.
- A custom legend (`GroundMapColor { rgb, material }`) maps your own colours to
  any `GroundMaterial`, for example a firmer sand. `GroundTypeMap::new` takes
  raw indices and a palette of up to 256 materials, and
  `GroundTypeMap::from_ground_types` takes a grid of `GroundType`s.
- Each contact manifold uses the material under its deepest point. Friction and
  restitution follow that material, and `GroundContactSample::ground_type`
  reports it, so a foot on a rock patch beside a mud pool is rigid and grippy
  while its neighbour sinks.
- A collider whose map has any soft ground is lowered by the deepest soil layer
  (see the implementation notes). Rigid patches still support contacts at the
  authored surface with plain-terrain friction and lever arms, but the contact
  geometry is generated differently, so replays are not bit-identical to an
  unmapped collider. Compare runs that use the same map.

## Deformable ground model

Sand and mud carry a `SoilDeformation`. A contact patch of radius `b` (the
touching ball, capsule or cylinder radius, at least `min_contact_radius_m`)
pressed to depth `z` is supported up to

```text
F_cap = A(z) · ((k_c / b + k_phi) · z^n + c · v_sink)
```

with the Bekker pressure-sinkage law, a viscous term `c`, and the spherical-cap
patch area `A(z) = π(2bz − z²)` (then `πb²` once `z ≥ b`). Sand uses Wong's
dry-sand moduli (`k_c` = 0.99 kN/m^(n+1), `k_phi` = 1528 kN/m^(n+2), `n` = 1.1).

- The ground yields when the load exceeds `F_cap`: the solver caps the contact's
  normal force there and the contact sinks further.
- Displaced ground stays displaced. Unloading leaves the contact at its depth,
  and lifting out leaves a hole: there is no elastic rebound.
- Mud creeps: under sustained load it keeps flowing at up to
  `creep_velocity_m_s` (0.03 m/s), scaled by the load relative to its static
  bearing, so a standing foot keeps sinking slowly.
- A firm layer at `max_sinkage_m` (sand 0.10 m, mud 0.12 m) supports any load.
- Friction is the surface coefficient combined as usual, capped at
  `max_friction`, plus `bulldozing_friction_gain` × sinkage/radius (saturating
  at one radius) for the ground pushed ahead of a sunken contact.

A sudden load overshoots the static equilibrium sinkage, because plastic ground
cannot push back; faster touchdowns sink deeper.

## Observing contacts

`RapierBackend::ground_contacts(world)` returns a `GroundContactSample` per
contact point after each step: ground type, point, ground normal, sinkage below
the undisturbed surface, sink speed, normal force, the bearing capacity the
solver used, the effective friction, and whether the contact was yielding. This
is simulation ground truth for validating terrain perception, with no noise or
latency. The normal force uses the same convention as `contact_points()`.

## Implementation notes (Rapier)

- The vendored Rapier solver accepts a per-contact `max_normal_force` (see
  `third_party/rapier3d/RNE_PATCH.md`); ground surfaces need the vendored crate.
- A deformable ground collider is moved down to its firm layer and given a
  contact skin of the soil depth. Contacts therefore start at the authored
  surface, and a shape that sinks into the soil never passes through the
  collider geometry (which would lose the contact on a height field). The
  backend measures contact depth from the authored surface (on a slope the
  vertical lowering is `depth · cos θ` along the face normal) and places each
  solver contact point at the authored surface, so support and friction act
  with the same lever arms as on plain terrain.
- Ray casts that hit deformable ground report the undisturbed surface, and
  `contact_separations()` subtracts contact skins.
- The skin also rounds the edges of neighbouring triangles, so a shape over or
  sunk into lowered ground touches those edges with tilted normals. While a
  shape has a face contact (normal along its triangle's own normal), its edge
  contacts are dropped; a shape resting only on edges, such as a real ridge,
  keeps them. This also removes the internal-edge snagging that plain height
  fields show.
- Per-contact soil state (displaced depth and accumulated creep) is kept in the
  contact manifold and resets when the manifold disappears.
