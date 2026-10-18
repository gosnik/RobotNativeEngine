# Heightmap terrain rendering

The renderer supports 16-bit grayscale PNG and little-endian R16 elevation
sources through `TerrainHeightMap`. Metric width/depth and height range are
explicit; columns run along +X, rows along +Z, and elevation along +Y. Eight-bit
previews and color images are rejected as elevation sources. Existing metric
height arrays can be shared directly with `TerrainHeightMap::new`.

`TerrainMesh` prepares retained mesh tiles at several resolutions. Camera
selection uses sampled elevation error projected into pixels. Global UVs allow
a landscape color bitmap or existing PBR texture maps to span the terrain;
normals use the full source grid so neighboring tiles share the same boundary
normals. Skirts end below the lowest source elevation to hide seams between
unlike LODs. Partial edge tiles are supported. Mesh storage is shared through
`Arc` and `RenderScene::item_from_shared_mesh`; selection does not rebuild meshes.

This design references [Terrain3D at 59bacc5](https://github.com/TokisanGames/Terrain3D/tree/59bacc5e61aede7d0fd96ae9cd263f4cc0f09858),
particularly its [fixed height data and view-dependent detail](https://github.com/TokisanGames/Terrain3D/blob/59bacc5e61aede7d0fd96ae9cd263f4cc0f09858/doc/docs/introduction.md)
and [high-precision heightmap import](https://github.com/TokisanGames/Terrain3D/blob/59bacc5e61aede7d0fd96ae9cd263f4cc0f09858/doc/docs/heightmaps.md).
It is an independent CPU-tiled implementation using RNE's existing renderer,
not a port of Terrain3D's Godot GPU clipmap, editing tools, or material-painting
system. LOD transitions are discrete and skirt-covered, not geomorphed.

## Render the supplied landscape

The [Motion Forge download 4960](https://www.motionforgepictures.com/?sdm_process_download=1&download_id=4960)
is `Rolling Hills Height Map 1k`. Use `Rolling Hills Height Map.png` for elevation
and `Rolling Hills Bitmap 1025.png` for color. The archive's PDF declares CC0.
The color bitmap includes baked shading; it is not a calibrated unlit albedo.
The sample uses a chosen 6 m square with 1.4 m relief, not artist-specified units.

```sh
cargo run -p rne_render_wgpu --release --example terrain_heightmap -- \
  'Rolling Hills Height Map.png' 'Rolling Hills Bitmap 1025.png' terrain.png
```

The example renders the full 1025 × 1025 elevation source through 256 cached
tiles. At its default overview it selects 31,168 triangles including skirts,
versus 2,097,152 surface triangles at full resolution (98.5% fewer triangles).
This is a geometry-count comparison, not a measured frame-rate claim.

## Physics independence

This module has no physics dependency and creates no colliders. Rendering LOD,
UVs, normals, materials and skirts must not be fed back into collision geometry.
Use the original authoritative elevation samples for collision. Applications
can choose a source resolution independently, but a visual-only upgrade must
not silently resample or rescale the physics world.

Hexapod uses its existing 129 × 129 Rough and 257 × 257 Extreme collision
heightfields unchanged. Its viewer uses the same metric samples for mesh
preparation, applies the supplied bitmap to Rough, and selects drawing detail
from the camera. The full-resolution overview example is separate from the
simulator. Collision accuracy, solver settings, timestep, and contact behavior
are not modified by the terrain rendering integration.

## Validation

Tests cover adjacent 16-bit values, PNG precision and rejected previews, partial
edge tiles, shared boundary normals/UVs, valid indices, lower distant triangle
counts, and retained mesh identity. The Hexapod viewer regression checks surface
positions, triangle winding, slope normals and metric extents against an
asymmetric heightfield. The example is also rendered through the actual wgpu
backend as a visual check.

Before/after half-second headless hold runs on both Rough and Extreme had
identical non-timing result fields and byte-identical joint, walking and stance
telemetry, with zero faults and finite results. This checks the rendering-only
integration's physics invariance; it is not a new gait or terrain qualification.
