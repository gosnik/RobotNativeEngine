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
  collider geometry (which would lose the contact on a height field). On a
  sloped height field the skin is applied along the face normal while the
  collider is lowered vertically, so the effective surface sits
  `depth · (1 − cos θ)` above the authored one (3 mm at 20° for 0.05 m).
- Ray casts that hit deformable ground report the undisturbed surface, and
  `contact_separations()` subtracts contact skins.
- When a shape sunk into the skin touches several height-field triangles, only
  the face whose normal best matches the ground's up axis (from the previous
  step) carries the contact; the others are skin-rounded edges.
- Per-contact soil state (displaced depth and accumulated creep) is kept in the
  contact manifold and resets when the manifold disappears.
