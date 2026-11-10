# Mesh hydrodynamics

`rne_physics::HydrodynamicMesh` evaluates water loads on a rigid or articulated
link. Supply closed, outward-wound indexed triangles in local metres. Use the
exterior geometry that displaces water and account for flooded cavities when
authoring the mesh. Contact shapes and render meshes can be authored separately.

At each physics step, call `evaluate` with the world transform of the mesh and
the link's center of mass, linear velocity at that center, and angular velocity.
The result is a world-frame force and a moment about the link center of mass.
Queue them with `PhysicsBackend::apply_external_body_wrench` at that center,
then step the physics backend. Repeat for each immersed link. Use a backend
with the `ExternalBodyWrench` capability.

The flat free surface lies at `Water::surface_y_m`. Buoyancy uses the volume
and centroid of the clipped mesh. Quadratic drag uses normal and tangential
velocity at each wetted triangle relative to a uniform current. Set the
gravity magnitude to match the world. Fit the drag coefficients for the
chosen geometry and operating range. A tilted surface can develop transverse
force during a depth change, and articulated motion changes the local water
loads. The model covers quasi-steady forces and hydrostatics. Added mass,
unsteady lift and wake coupling require further models.

UUV Simulator provides a reference for link-local buoyancy and
relative-current damping. Its Fossen model uses configured damping and
added-mass coefficients. RNE's drag is integrated over wetted mesh triangles.
[UUV Simulator configuration](https://github.com/uuvsimulator/uuv_simulator/wiki/Explaining-the-configuration-of-the-ROV-model-description)

```sh
cargo test -p rne_physics --lib
cargo test -p rne_physics_rapier --test underwater
```
