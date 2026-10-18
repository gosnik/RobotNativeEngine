//! Precision-preserving heightmap terrain with cached, camera-selected mesh tiles.
//!
//! Like Terrain3D, elevation data stays fixed while view-dependent detail changes.
//! This implementation uses cached CPU mesh tiles and skirts, rather than Godot's
//! GPU clipmap. Columns map to +X, rows to +Z, heights to +Y, with a centered extent.
use crate::TriangleMesh;
use rne_math::Vec3;
use std::sync::Arc;
use thiserror::Error;

/// Invalid terrain input or image decoding failure.
#[derive(Clone, Debug, Error, PartialEq)]
#[error("invalid terrain: {0}")]
pub struct TerrainError(pub String);

/// Immutable metric elevation samples shared by rendering and collision callers.
#[derive(Clone, Debug)]
pub struct TerrainHeightMap {
    columns: u32,
    rows: u32,
    heights_m: Arc<[f64]>,
    extent_m: [f64; 2],
}

impl TerrainHeightMap {
    /// Creates a centered X/Z heightmap from row-major elevations in metres.
    pub fn new(
        columns: u32,
        rows: u32,
        heights_m: Arc<[f64]>,
        extent_m: [f64; 2],
    ) -> Result<Self, TerrainError> {
        let count = (columns as usize).checked_mul(rows as usize);
        if columns < 2
            || rows < 2
            || count != Some(heights_m.len())
            || heights_m
                .iter()
                .any(|h| !h.is_finite() || !(*h as f32).is_finite())
            || extent_m
                .iter()
                .any(|v| !v.is_finite() || (*v as f32) <= 0.0 || !(*v as f32).is_finite())
        {
            return Err(TerrainError(
                "dimensions, elevations, or metric extent are invalid".into(),
            ));
        }
        Ok(Self {
            columns,
            rows,
            heights_m,
            extent_m,
        })
    }

    /// Decodes unsigned little-endian 16-bit samples without reducing precision.
    /// Values 0 and 65535 map to the supplied minimum and maximum heights.
    pub fn from_r16_le(
        bytes: &[u8],
        columns: u32,
        rows: u32,
        extent_m: [f64; 2],
        height_range_m: [f64; 2],
    ) -> Result<Self, TerrainError> {
        let expected = (columns as usize)
            .checked_mul(rows as usize)
            .and_then(|v| v.checked_mul(2));
        if expected != Some(bytes.len()) {
            return Err(TerrainError(
                "R16 byte count does not match dimensions".into(),
            ));
        }
        Self::from_u16(
            bytes
                .chunks_exact(2)
                .map(|p| u16::from_le_bytes([p[0], p[1]])),
            columns,
            rows,
            extent_m,
            height_range_m,
        )
    }

    /// Imports a 16-bit grayscale PNG. Eight-bit previews and color bitmaps are
    /// rejected so they cannot silently replace the elevation source.
    pub fn from_png16(
        bytes: &[u8],
        extent_m: [f64; 2],
        height_range_m: [f64; 2],
    ) -> Result<Self, TerrainError> {
        let image = image::load_from_memory_with_format(bytes, image::ImageFormat::Png)
            .map_err(|e| TerrainError(e.to_string()))?;
        let image::DynamicImage::ImageLuma16(image) = image else {
            return Err(TerrainError("height PNG must be 16-bit grayscale".into()));
        };
        let (columns, rows) = image.dimensions();
        Self::from_u16(
            image.into_raw().into_iter(),
            columns,
            rows,
            extent_m,
            height_range_m,
        )
    }

    fn from_u16(
        samples: impl Iterator<Item = u16>,
        columns: u32,
        rows: u32,
        extent_m: [f64; 2],
        range: [f64; 2],
    ) -> Result<Self, TerrainError> {
        if !range[0].is_finite()
            || !range[1].is_finite()
            || range[1] < range[0]
            || !(range[1] - range[0]).is_finite()
        {
            return Err(TerrainError(
                "height range must be finite and ordered".into(),
            ));
        }
        let heights = samples
            .map(|v| range[0] + f64::from(v) / 65535.0 * (range[1] - range[0]))
            .collect::<Vec<_>>();
        Self::new(columns, rows, heights.into(), extent_m)
    }

    /// Number of samples along X.
    pub fn columns(&self) -> u32 {
        self.columns
    }
    /// Number of samples along Z.
    pub fn rows(&self) -> u32 {
        self.rows
    }
    /// Shared row-major elevations in metres.
    pub fn heights_m(&self) -> &Arc<[f64]> {
        &self.heights_m
    }
    /// Full width and depth in metres.
    pub fn extent_m(&self) -> [f64; 2] {
        self.extent_m
    }

    fn height(&self, x: u32, z: u32) -> f64 {
        self.heights_m[z as usize * self.columns as usize + x as usize]
    }
    fn position(&self, x: u32, z: u32) -> Vec3 {
        Vec3::new(
            (f64::from(x) / f64::from(self.columns - 1) - 0.5) * self.extent_m[0],
            self.height(x, z),
            (f64::from(z) / f64::from(self.rows - 1) - 0.5) * self.extent_m[1],
        )
    }
    fn normal(&self, x: u32, z: u32) -> [f32; 3] {
        let x0 = x.saturating_sub(1);
        let x1 = (x + 1).min(self.columns - 1);
        let z0 = z.saturating_sub(1);
        let z1 = (z + 1).min(self.rows - 1);
        let dx = self.position(x1, z) - self.position(x0, z);
        let dz = self.position(x, z1) - self.position(x, z0);
        dz.cross(dx).normalize().to_array().map(|v| v as f32)
    }
}

#[derive(Clone, Debug)]
struct TerrainLod {
    mesh: Arc<TriangleMesh>,
    error_m: f64,
}
#[derive(Clone, Debug)]
struct TerrainTile {
    center: Vec3,
    radius_m: f64,
    levels: Vec<TerrainLod>,
}

/// Retained mesh tiles with globally consistent normals and UVs at every LOD.
///
/// Meshes are built once and selected by projected sampled elevation error. Vertical
/// skirts cover cracks between unlike neighboring levels. They do not alter
/// the authoritative heightmap or physics. No topology is rebuilt per frame.
#[derive(Clone, Debug)]
pub struct TerrainMesh {
    tiles: Vec<TerrainTile>,
}

impl TerrainMesh {
    /// Prepares power-of-two tiles (2–256 cells per side) and their LODs.
    /// Skirts end `skirt_depth_m` below the map's lowest elevation, covering
    /// arbitrary neighboring LOD seams and giving the outer border a level base.
    pub fn new(
        map: &TerrainHeightMap,
        tile_cells: u32,
        skirt_depth_m: f64,
    ) -> Result<Self, TerrainError> {
        let min = map.heights_m.iter().copied().fold(f64::INFINITY, f64::min);
        if !(2..=256).contains(&tile_cells)
            || !tile_cells.is_power_of_two()
            || !skirt_depth_m.is_finite()
            || skirt_depth_m <= 0.0
            || !((min - skirt_depth_m) as f32).is_finite()
        {
            return Err(TerrainError(
                "tile size or seam-covering skirt depth is invalid".into(),
            ));
        }
        let mut tiles = Vec::new();
        for z0 in (0..map.rows - 1).step_by(tile_cells as usize) {
            for x0 in (0..map.columns - 1).step_by(tile_cells as usize) {
                let x1 = x0.saturating_add(tile_cells).min(map.columns - 1);
                let z1 = z0.saturating_add(tile_cells).min(map.rows - 1);
                let mut low = map.position(x0, z0);
                let mut high = low;
                for z in z0..=z1 {
                    for x in x0..=x1 {
                        let p = map.position(x, z);
                        low = low.min(p);
                        high = high.max(p);
                    }
                }
                let center = (low + high) * 0.5;
                let radius_m = (high - low).length() * 0.5;
                let mut levels = Vec::new();
                for power in 0..=tile_cells.ilog2() {
                    let step = 1 << power;
                    let xs = sample_axis(x0, x1, step);
                    let zs = sample_axis(z0, z1, step);
                    let error_m = approximation_error(map, &xs, &zs);
                    let mesh = Arc::new(tile_mesh(map, &xs, &zs, min - skirt_depth_m));
                    levels.push(TerrainLod { mesh, error_m });
                    if xs.len() == 2 && zs.len() == 2 {
                        break;
                    }
                }
                tiles.push(TerrainTile {
                    center,
                    radius_m,
                    levels,
                });
            }
        }
        Ok(Self { tiles })
    }

    /// Selects cached meshes using a local-space camera and vertical projection.
    /// The camera must use the same metric coordinates as the heightmap; callers
    /// with transformed terrain should first transform it into terrain space.
    /// Stable row-major tile order makes repeated selections deterministic.
    pub fn meshes_for_view(
        &self,
        camera_local_m: Vec3,
        vertical_fov_rad: f64,
        viewport_height: u32,
        max_error_pixels: f64,
    ) -> Result<Vec<Arc<TriangleMesh>>, TerrainError> {
        if !camera_local_m.is_finite()
            || !vertical_fov_rad.is_finite()
            || !(0.0..std::f64::consts::PI).contains(&vertical_fov_rad)
            || vertical_fov_rad == 0.0
            || viewport_height == 0
            || !max_error_pixels.is_finite()
            || max_error_pixels <= 0.0
        {
            return Err(TerrainError(
                "camera projection or pixel error is invalid".into(),
            ));
        }
        let projection = f64::from(viewport_height) / (2.0 * (vertical_fov_rad * 0.5).tan());
        Ok(self
            .tiles
            .iter()
            .map(|tile| {
                let distance = ((camera_local_m - tile.center).length() - tile.radius_m).max(1e-6);
                tile.levels
                    .iter()
                    .rev()
                    .find(|lod| lod.error_m * projection / distance <= max_error_pixels)
                    .unwrap_or(&tile.levels[0])
                    .mesh
                    .clone()
            })
            .collect())
    }

    /// Returns every tile at full resolution, for exact geometry comparisons.
    pub fn finest_meshes(&self) -> Vec<Arc<TriangleMesh>> {
        self.tiles
            .iter()
            .map(|t| t.levels[0].mesh.clone())
            .collect()
    }
}

fn sample_axis(start: u32, end: u32, step: u32) -> Vec<u32> {
    let mut values = (start..end).step_by(step as usize).collect::<Vec<_>>();
    values.push(end);
    values
}
fn approximation_error(map: &TerrainHeightMap, xs: &[u32], zs: &[u32]) -> f64 {
    let mut error = 0.0_f64;
    for z in zs.windows(2) {
        for x in xs.windows(2) {
            let a = map.height(x[0], z[0]);
            let b = map.height(x[1], z[0]);
            let c = map.height(x[0], z[1]);
            let d = map.height(x[1], z[1]);
            for row in z[0]..=z[1] {
                for column in x[0]..=x[1] {
                    let u = f64::from(column - x[0]) / f64::from(x[1] - x[0]);
                    let v = f64::from(row - z[0]) / f64::from(z[1] - z[0]);
                    let height = if u + v <= 1.0 {
                        a + (b - a) * u + (c - a) * v
                    } else {
                        d + (c - d) * (1.0 - u) + (b - d) * (1.0 - v)
                    };
                    error = error.max((height - map.height(column, row)).abs());
                }
            }
        }
    }
    error
}
fn tile_mesh(map: &TerrainHeightMap, xs: &[u32], zs: &[u32], skirt_bottom_m: f64) -> TriangleMesh {
    let mut mesh = TriangleMesh {
        positions: Vec::new(),
        normals: Vec::new(),
        texcoords: Vec::new(),
        indices: Vec::new(),
        skinning: None,
    };
    for &z in zs {
        for &x in xs {
            mesh.positions
                .push(map.position(x, z).to_array().map(|v| v as f32));
            mesh.normals.push(map.normal(x, z));
            mesh.texcoords.push([
                x as f32 / (map.columns - 1) as f32,
                z as f32 / (map.rows - 1) as f32,
            ]);
        }
    }
    let nx = xs.len() as u32;
    let nz = zs.len() as u32;
    for z in 0..nz - 1 {
        for x in 0..nx - 1 {
            let a = z * nx + x;
            mesh.indices
                .extend_from_slice(&[a, a + nx, a + 1, a + 1, a + nx, a + nx + 1]);
        }
    }
    let edge = (0..nx)
        .chain((1..nz).map(|z| z * nx + nx - 1))
        .chain((0..nx - 1).rev().map(|x| (nz - 1) * nx + x))
        .chain((1..nz - 1).rev().map(|z| z * nx))
        .collect::<Vec<_>>();
    for i in 0..edge.len() {
        let a = edge[i] as usize;
        let b = edge[(i + 1) % edge.len()] as usize;
        let pa = Vec3::from_array(mesh.positions[a].map(f64::from));
        let pb = Vec3::from_array(mesh.positions[b].map(f64::from));
        let normal = (pb - pa)
            .cross(-Vec3::Y)
            .normalize()
            .to_array()
            .map(|v| v as f32);
        let first = mesh.positions.len() as u32;
        for (index, p) in [
            (a, pa),
            (b, pb),
            (a, Vec3::new(pa.x, skirt_bottom_m, pa.z)),
            (b, Vec3::new(pb.x, skirt_bottom_m, pb.z)),
        ] {
            mesh.positions.push(p.to_array().map(|v| v as f32));
            mesh.normals.push(normal);
            mesh.texcoords.push(mesh.texcoords[index]);
        }
        mesh.indices.extend_from_slice(&[
            first,
            first + 1,
            first + 2,
            first + 1,
            first + 3,
            first + 2,
        ]);
    }
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn r16_preserves_adjacent_values_and_rejects_invalid_input() {
        let values = [0_u16, 1, 65534, 65535];
        let bytes = values
            .into_iter()
            .flat_map(u16::to_le_bytes)
            .collect::<Vec<_>>();
        let map = TerrainHeightMap::from_r16_le(&bytes, 2, 2, [2.0, 4.0], [-1.0, 1.0]).unwrap();
        assert_eq!(map.heights_m[0], -1.0);
        assert_eq!(map.heights_m[3], 1.0);
        assert!(map.heights_m[1] > map.heights_m[0]);
        assert!(TerrainHeightMap::from_r16_le(&bytes[..7], 2, 2, [2.0, 4.0], [-1.0, 1.0]).is_err());
        assert!(TerrainHeightMap::new(2, 2, vec![f64::NAN; 4].into(), [2.0, 4.0]).is_err());
    }
    #[test]
    fn png_import_preserves_sixteen_bits_and_rejects_previews() {
        let image =
            image::ImageBuffer::<image::Luma<u16>, _>::from_raw(2, 2, vec![0, 1, 65534, 65535])
                .unwrap();
        let mut bytes = std::io::Cursor::new(Vec::new());
        image::DynamicImage::ImageLuma16(image)
            .write_to(&mut bytes, image::ImageFormat::Png)
            .unwrap();
        let map =
            TerrainHeightMap::from_png16(bytes.get_ref(), [2.0, 2.0], [0.0, 65535.0]).unwrap();
        assert_eq!(map.heights_m.as_ref(), &[0.0, 1.0, 65534.0, 65535.0]);
        let mut bytes = std::io::Cursor::new(Vec::new());
        image::DynamicImage::new_luma8(2, 2)
            .write_to(&mut bytes, image::ImageFormat::Png)
            .unwrap();
        assert!(TerrainHeightMap::from_png16(bytes.get_ref(), [2.0, 2.0], [0.0, 1.0]).is_err());
    }

    #[test]
    fn tile_boundaries_share_global_normals_uvs_and_finest_heights() {
        let h = (0..5)
            .flat_map(|z| (0..7).map(move |x| f64::from(x * x + z)))
            .collect::<Vec<_>>();
        let map = TerrainHeightMap::new(7, 5, h.into(), [6.0, 4.0]).unwrap();
        let terrain = TerrainMesh::new(&map, 4, 50.0).unwrap();
        let meshes = terrain.finest_meshes();
        assert_eq!(meshes.len(), 2);
        // First tile has 5 columns, second has 3, including the shared seam.
        for row in 0..5 {
            let a = row * 5 + 4;
            let b = row * 3;
            assert_eq!(meshes[0].positions[a], meshes[1].positions[b]);
            assert_eq!(meshes[0].normals[a], meshes[1].normals[b]);
            assert_eq!(meshes[0].texcoords[a], meshes[1].texcoords[b]);
        }
        assert_eq!(meshes[1].positions[14], [3.0, 40.0, 2.0]);
        for mesh in meshes {
            assert!(mesh
                .indices
                .iter()
                .all(|&i| (i as usize) < mesh.positions.len()));
            assert!(mesh.normals.iter().all(|n| n.iter().all(|v| v.is_finite())));
        }
    }
    #[test]
    fn distant_views_select_fewer_triangles_and_reuse_meshes() {
        let h = (0..17)
            .flat_map(|z| (0..17).map(move |x| ((x * z) as f64).sin()))
            .collect::<Vec<_>>();
        let map = TerrainHeightMap::new(17, 17, h.into(), [16.0, 16.0]).unwrap();
        let terrain = TerrainMesh::new(&map, 8, 3.0).unwrap();
        let near = terrain.meshes_for_view(Vec3::ZERO, 1.0, 720, 1.0).unwrap();
        let far = terrain
            .meshes_for_view(Vec3::splat(10000.0), 1.0, 720, 1.0)
            .unwrap();
        let again = terrain
            .meshes_for_view(Vec3::splat(10000.0), 1.0, 720, 1.0)
            .unwrap();
        assert!(
            far.iter().map(|m| m.triangle_count()).sum::<usize>()
                < near.iter().map(|m| m.triangle_count()).sum::<usize>()
        );
        assert!(Arc::ptr_eq(&far[0], &again[0]));
        assert!(terrain.meshes_for_view(Vec3::ZERO, 0.0, 720, 1.0).is_err());
    }
}
