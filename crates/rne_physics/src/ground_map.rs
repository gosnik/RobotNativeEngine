//! Per-point ground materials for mixed terrain.
//!
//! A [`GroundTypeMap`] is a grid of material indices laid over a terrain
//! collider's horizontal footprint, usually painted as an image that matches
//! the terrain's heightmap pixel for pixel. Attach it through
//! [`crate::GroundSurface::mapped`].

use std::collections::HashMap;
use std::io::Cursor;
use std::sync::Arc;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::components::{GroundMaterial, GroundType};

/// Grid of ground materials over a collider's local XZ footprint.
///
/// Columns run along local +X and rows along local +Z, row-major, the same
/// layout as [`crate::ColliderShape::HeightField`] heights. The grid is
/// stretched over the collider's footprint (for a height field, its full
/// `scale.x` by `scale.z` extent) with samples at the grid *vertices*: sample
/// `(0, 0)` sits at the footprint's −X/−Z corner and sample
/// `(columns − 1, rows − 1)` at its +X/+Z corner. A map with the same size as a
/// height field therefore assigns one material per height sample, and a point
/// takes the material of the nearest sample. Maps of other resolutions are
/// stretched the same way; points outside the footprint take the nearest edge
/// sample.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundTypeMap {
    columns: u32,
    rows: u32,
    cells: Arc<[u16]>,
    materials: Arc<[GroundMaterial]>,
}

/// Why a [`GroundTypeMap`] could not be built.
#[derive(Debug, Error, PartialEq)]
pub enum GroundTypeMapError {
    /// The grid has no columns or no rows.
    #[error("ground map must have at least one column and one row, got {columns}x{rows}")]
    EmptyGrid {
        /// Requested columns.
        columns: u32,
        /// Requested rows.
        rows: u32,
    },
    /// The number of cells (or pixel bytes) does not match the grid size.
    #[error("ground map {columns}x{rows} expects {expected} values, got {actual}")]
    SizeMismatch {
        /// Grid columns.
        columns: u32,
        /// Grid rows.
        rows: u32,
        /// Expected value count.
        expected: usize,
        /// Supplied value count.
        actual: usize,
    },
    /// The material palette is empty or has more than 65 536 entries.
    #[error("ground map needs 1 to 65536 materials, got {0}")]
    MaterialCount(usize),
    /// A palette material fails [`GroundMaterial::is_valid`].
    #[error("ground map material {0} is invalid")]
    InvalidMaterial(usize),
    /// A cell refers to a material the palette does not have.
    #[error("ground map cell {cell} uses material {index}, but only {materials} exist")]
    IndexOutOfRange {
        /// Row-major cell index.
        cell: usize,
        /// Material index found in the cell.
        index: u16,
        /// Palette length.
        materials: usize,
    },
    /// A colour legend is empty.
    #[error("ground map colour legend is empty")]
    EmptyLegend,
    /// The image could not be decoded.
    #[error("ground map image: {0}")]
    Image(String),
}

/// Colour that stands for one material in a painted ground map.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundMapColor {
    /// sRGB colour, as painted.
    pub rgb: [u8; 3],
    /// Material for pixels nearest this colour.
    pub material: GroundMaterial,
}

impl GroundType {
    /// Colour of this ground kind in a painted map (see
    /// [`GroundTypeMap::default_legend`]).
    ///
    /// * Normal: green `#288C28`
    /// * Rock: grey `#808080`
    /// * Sand: yellow `#E6C850`
    /// * Mud: brown `#643C14`
    pub fn map_color_rgb(self) -> [u8; 3] {
        match self {
            GroundType::Normal => [0x28, 0x8C, 0x28],
            GroundType::Rock => [0x80, 0x80, 0x80],
            GroundType::Sand => [0xE6, 0xC8, 0x50],
            GroundType::Mud => [0x64, 0x3C, 0x14],
        }
    }
}

/// RGB parameter encoding of a ground map, for maps that carry friction and
/// soil properties as well as the ground type.
///
/// | Channel | Stores | Encoding |
/// |---|---|---|
/// | R | ground type | normal 0, rock 85, sand 170, mud 255 (nearest level wins) |
/// | G | surface friction | linear, 0 = [`Self::friction_min`], 255 = [`Self::friction_max`] (default 0–2) |
/// | B | deformability, creep | upper 4 bits `d`, lower 4 bits `c` |
///
/// Each pixel starts from the type's [`GroundMaterial::preset`]:
///
/// * G replaces the surface friction. For sand and mud the friction cap
///   ([`GroundMaterial::max_friction`]) scales by the same ratio, keeping the
///   preset's ratio between surface and internal friction.
/// * `d` (0–15) scales the soil's Bekker moduli `k_c` and `k_phi` by
///   `2^((8 − d) / 2)`: 8 is the preset, 0 is 16× stiffer, 15 about 11×
///   softer.
/// * `c` (0–15) sets the creep speed to `c ×` [`Self::creep_step_m_s`]
///   (default 3.75 mm/s, so 8 is mud's preset 0.03 m/s and 0 is no creep).
/// * Normal and rock are rigid and ignore B.
///
/// [`Self::encode`] gives the pixel for a material, for example
/// `encode(&GroundMaterial::preset(GroundType::Sand))` is `[170, 64, 0x80]`.
/// Paint G and B with hard-edged brushes: blended B values mix the two nibbles.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundMapEncoding {
    /// Friction for G = 0.
    pub friction_min: f32,
    /// Friction for G = 255.
    pub friction_max: f32,
    /// Creep speed per step of the creep nibble, in m/s.
    pub creep_step_m_s: f64,
}

impl Default for GroundMapEncoding {
    fn default() -> Self {
        Self {
            friction_min: 0.0,
            friction_max: 2.0,
            creep_step_m_s: 0.00375,
        }
    }
}

impl GroundMapEncoding {
    /// Ground type of an R value: the nearest of 0, 85, 170 and 255.
    pub fn decode_type(red: u8) -> GroundType {
        GroundType::ALL[usize::from((u16::from(red) + 42) / 85).min(3)]
    }

    /// R value of a ground type.
    pub fn type_level(ground_type: GroundType) -> u8 {
        match ground_type {
            GroundType::Normal => 0,
            GroundType::Rock => 85,
            GroundType::Sand => 170,
            GroundType::Mud => 255,
        }
    }

    /// Material of one encoded pixel.
    pub fn decode(&self, [red, green, blue]: [u8; 3]) -> GroundMaterial {
        let ground_type = Self::decode_type(red);
        let preset = GroundMaterial::preset(ground_type);
        let friction = (self.friction_min
            + (self.friction_max - self.friction_min) * f32::from(green) / 255.0)
            .max(0.0);
        let mut material = GroundMaterial { friction, ..preset };
        if let Some(mut soil) = preset.deformation {
            if preset.friction > 0.0 && preset.max_friction.is_finite() {
                material.max_friction = preset.max_friction * friction / preset.friction;
            }
            let stiffness = 2f64.powf((8.0 - f64::from(blue >> 4)) / 2.0);
            soil.cohesive_modulus_n_per_m_n1 *= stiffness;
            soil.frictional_modulus_n_per_m_n2 *= stiffness;
            soil.creep_velocity_m_s = f64::from(blue & 0x0F) * self.creep_step_m_s.max(0.0);
            material.deformation = Some(soil);
        }
        material
    }

    /// Nearest encoded pixel for a material (exact for the type and, within
    /// one step, for friction, deformability and creep).
    pub fn encode(&self, material: &GroundMaterial) -> [u8; 3] {
        let span = self.friction_max - self.friction_min;
        let green = if span > 0.0 {
            ((material.friction - self.friction_min) / span * 255.0)
                .round()
                .clamp(0.0, 255.0) as u8
        } else {
            0
        };
        let preset = GroundMaterial::preset(material.ground_type);
        let blue = match (material.deformation, preset.deformation) {
            (Some(soil), Some(nominal)) => {
                let ratio =
                    soil.frictional_modulus_n_per_m_n2 / nominal.frictional_modulus_n_per_m_n2;
                let d = (8.0 - 2.0 * ratio.log2()).round().clamp(0.0, 15.0) as u8;
                let c = if self.creep_step_m_s > 0.0 {
                    (soil.creep_velocity_m_s / self.creep_step_m_s)
                        .round()
                        .clamp(0.0, 15.0) as u8
                } else {
                    0
                };
                (d << 4) | c
            }
            _ => 0x80,
        };
        [Self::type_level(material.ground_type), green, blue]
    }
}

impl GroundTypeMap {
    /// Builds a map from row-major material indices and their palette.
    pub fn new(
        columns: u32,
        rows: u32,
        cells: impl Into<Arc<[u16]>>,
        materials: impl Into<Arc<[GroundMaterial]>>,
    ) -> Result<Self, GroundTypeMapError> {
        let map = Self {
            columns,
            rows,
            cells: cells.into(),
            materials: materials.into(),
        };
        map.validate()?;
        Ok(map)
    }

    /// Builds a map of the [`GroundMaterial::preset`] of each cell's kind.
    pub fn from_ground_types(
        columns: u32,
        rows: u32,
        types: &[GroundType],
    ) -> Result<Self, GroundTypeMapError> {
        let cells: Vec<u16> = types
            .iter()
            .map(|kind| GroundType::ALL.iter().position(|k| k == kind).unwrap_or(0) as u16)
            .collect();
        let materials: Vec<GroundMaterial> = GroundType::ALL
            .into_iter()
            .map(GroundMaterial::preset)
            .collect();
        Self::new(columns, rows, cells, materials)
    }

    /// The four preset materials with their [`GroundType::map_color_rgb`].
    pub fn default_legend() -> Vec<GroundMapColor> {
        GroundType::ALL
            .into_iter()
            .map(|kind| GroundMapColor {
                rgb: kind.map_color_rgb(),
                material: GroundMaterial::preset(kind),
            })
            .collect()
    }

    /// Builds a map from packed 8-bit pixels with `channels` bytes per pixel
    /// (1 grey, 2 grey+alpha, 3 RGB, 4 RGBA; alpha is ignored), giving each
    /// pixel the legend material whose colour is nearest. Nearest matching
    /// keeps anti-aliased brush edges usable.
    pub fn from_pixels(
        columns: u32,
        rows: u32,
        pixels: &[u8],
        channels: usize,
        legend: &[GroundMapColor],
    ) -> Result<Self, GroundTypeMapError> {
        if legend.is_empty() {
            return Err(GroundTypeMapError::EmptyLegend);
        }
        if !(1..=4).contains(&channels) {
            return Err(GroundTypeMapError::Image(format!(
                "unsupported channel count {channels}"
            )));
        }
        let expected = columns as usize * rows as usize * channels;
        if pixels.len() != expected {
            return Err(GroundTypeMapError::SizeMismatch {
                columns,
                rows,
                expected,
                actual: pixels.len(),
            });
        }
        let cells: Vec<u16> = pixels
            .chunks_exact(channels)
            .map(|pixel| {
                let rgb = if channels >= 3 {
                    [pixel[0], pixel[1], pixel[2]]
                } else {
                    [pixel[0]; 3]
                };
                nearest_color(legend, rgb)
            })
            .collect();
        let materials: Vec<GroundMaterial> = legend.iter().map(|entry| entry.material).collect();
        Self::new(columns, rows, cells, materials)
    }

    /// Decodes a PNG (any colour type or bit depth) and maps its colours with
    /// [`Self::from_pixels`]. Image columns run along +X and rows along +Z.
    pub fn from_png(
        png_bytes: &[u8],
        legend: &[GroundMapColor],
    ) -> Result<Self, GroundTypeMapError> {
        let image = decode_png(png_bytes)?;
        Self::from_pixels(
            image.width,
            image.height,
            &image.pixels,
            image.channels,
            legend,
        )
    }

    /// Builds a map from packed 8-bit pixels in the RGB parameter encoding
    /// (see [`GroundMapEncoding`]), with `channels` bytes per pixel (3 RGB or
    /// 4 RGBA; alpha is ignored).
    pub fn from_encoded_pixels(
        columns: u32,
        rows: u32,
        pixels: &[u8],
        channels: usize,
        encoding: &GroundMapEncoding,
    ) -> Result<Self, GroundTypeMapError> {
        if !(3..=4).contains(&channels) {
            return Err(GroundTypeMapError::Image(format!(
                "the RGB ground encoding needs RGB or RGBA pixels, got {channels} channels"
            )));
        }
        let expected = columns as usize * rows as usize * channels;
        if pixels.len() != expected {
            return Err(GroundTypeMapError::SizeMismatch {
                columns,
                rows,
                expected,
                actual: pixels.len(),
            });
        }
        let mut palette: HashMap<[u8; 3], u16> = HashMap::new();
        let mut materials = Vec::new();
        let mut cells = Vec::with_capacity(columns as usize * rows as usize);
        for pixel in pixels.chunks_exact(channels) {
            // Snap R to its type level, so equal materials share an entry.
            let key = [
                GroundMapEncoding::type_level(GroundMapEncoding::decode_type(pixel[0])),
                pixel[1],
                pixel[2],
            ];
            let index = match palette.get(&key) {
                Some(&index) => index,
                None => {
                    let index = u16::try_from(materials.len())
                        .map_err(|_| GroundTypeMapError::MaterialCount(materials.len() + 1))?;
                    materials.push(encoding.decode(key));
                    palette.insert(key, index);
                    index
                }
            };
            cells.push(index);
        }
        Self::new(columns, rows, cells, materials)
    }

    /// Decodes a PNG in the RGB parameter encoding with
    /// [`Self::from_encoded_pixels`]. Image columns run along +X and rows along
    /// +Z.
    pub fn from_encoded_png(
        png_bytes: &[u8],
        encoding: &GroundMapEncoding,
    ) -> Result<Self, GroundTypeMapError> {
        let image = decode_png(png_bytes)?;
        Self::from_encoded_pixels(
            image.width,
            image.height,
            &image.pixels,
            image.channels,
            encoding,
        )
    }

    /// Grid columns (along local +X).
    pub fn columns(&self) -> u32 {
        self.columns
    }

    /// Grid rows (along local +Z).
    pub fn rows(&self) -> u32 {
        self.rows
    }

    /// Row-major material indices.
    pub fn cells(&self) -> &[u16] {
        &self.cells
    }

    /// Material palette indexed by [`Self::cells`].
    pub fn materials(&self) -> &[GroundMaterial] {
        &self.materials
    }

    /// Material index of the sample nearest normalised footprint coordinates
    /// `u` (0 at −X, 1 at +X) and `v` (0 at −Z, 1 at +Z), clamped to the map.
    pub fn index_at(&self, u: f64, v: f64) -> u16 {
        let sample = |t: f64, count: u32| -> usize {
            let last = count.saturating_sub(1) as f64;
            let t = if t.is_finite() { t } else { 0.0 };
            (t.clamp(0.0, 1.0) * last).round() as usize
        };
        let column = sample(u, self.columns);
        let row = sample(v, self.rows);
        self.cells[row * self.columns as usize + column]
    }

    /// Material at normalised footprint coordinates (see [`Self::index_at`]).
    pub fn material_at(&self, u: f64, v: f64) -> &GroundMaterial {
        &self.materials[usize::from(self.index_at(u, v))]
    }

    /// Deepest soil layer among the palette's materials, in metres (0 when
    /// every material is rigid).
    pub fn max_sinkage_m(&self) -> f64 {
        self.materials
            .iter()
            .filter_map(|material| material.deformation)
            .map(|soil| soil.max_sinkage_m)
            .fold(0.0, f64::max)
    }

    /// Whether the map is well formed (always true for maps built by this
    /// type's constructors; checks deserialised maps).
    pub fn is_valid(&self) -> bool {
        self.validate().is_ok()
    }

    fn validate(&self) -> Result<(), GroundTypeMapError> {
        if self.columns == 0 || self.rows == 0 {
            return Err(GroundTypeMapError::EmptyGrid {
                columns: self.columns,
                rows: self.rows,
            });
        }
        let expected = self.columns as usize * self.rows as usize;
        if self.cells.len() != expected {
            return Err(GroundTypeMapError::SizeMismatch {
                columns: self.columns,
                rows: self.rows,
                expected,
                actual: self.cells.len(),
            });
        }
        if self.materials.is_empty() || self.materials.len() > usize::from(u16::MAX) + 1 {
            return Err(GroundTypeMapError::MaterialCount(self.materials.len()));
        }
        if let Some(index) = self.materials.iter().position(|m| !m.is_valid()) {
            return Err(GroundTypeMapError::InvalidMaterial(index));
        }
        if let Some((cell, &index)) = self
            .cells
            .iter()
            .enumerate()
            .find(|(_, &index)| usize::from(index) >= self.materials.len())
        {
            return Err(GroundTypeMapError::IndexOutOfRange {
                cell,
                index,
                materials: self.materials.len(),
            });
        }
        Ok(())
    }
}

struct DecodedImage {
    width: u32,
    height: u32,
    channels: usize,
    pixels: Vec<u8>,
}

/// Decodes any PNG to packed 8-bit samples.
fn decode_png(png_bytes: &[u8]) -> Result<DecodedImage, GroundTypeMapError> {
    let image_error = |error: png::DecodingError| GroundTypeMapError::Image(error.to_string());
    let mut decoder = png::Decoder::new(Cursor::new(png_bytes));
    decoder.set_transformations(png::Transformations::normalize_to_color8());
    let mut reader = decoder.read_info().map_err(image_error)?;
    let size = reader
        .output_buffer_size()
        .ok_or_else(|| GroundTypeMapError::Image("image too large".to_owned()))?;
    let mut buffer = vec![0; size];
    let frame = reader.next_frame(&mut buffer).map_err(image_error)?;
    let channels = frame.color_type.samples();
    let row_bytes = frame.width as usize * channels;
    let mut pixels = Vec::with_capacity(row_bytes * frame.height as usize);
    for line in buffer.chunks(frame.line_size).take(frame.height as usize) {
        pixels.extend_from_slice(&line[..row_bytes]);
    }
    Ok(DecodedImage {
        width: frame.width,
        height: frame.height,
        channels,
        pixels,
    })
}

fn nearest_color(legend: &[GroundMapColor], rgb: [u8; 3]) -> u16 {
    let distance = |entry: &GroundMapColor| -> u32 {
        entry
            .rgb
            .iter()
            .zip(rgb)
            .map(|(&a, b)| (i32::from(a) - i32::from(b)).unsigned_abs().pow(2))
            .sum()
    };
    legend
        .iter()
        .enumerate()
        .min_by_key(|(_, entry)| distance(entry))
        .map_or(0, |(index, _)| index as u16)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_png(width: u32, height: u32, color: png::ColorType, data: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        {
            let mut encoder = png::Encoder::new(&mut bytes, width, height);
            encoder.set_color(color);
            encoder.set_depth(png::BitDepth::Eight);
            let mut writer = encoder.write_header().unwrap();
            writer.write_image_data(data).unwrap();
        }
        bytes
    }

    #[test]
    fn samples_nearest_vertex_like_a_height_field() {
        use GroundType::*;
        // 3x2 grid: row 0 (−Z) normal rock sand, row 1 (+Z) mud mud rock.
        let map =
            GroundTypeMap::from_ground_types(3, 2, &[Normal, Rock, Sand, Mud, Mud, Rock]).unwrap();
        let kind = |u, v| map.material_at(u, v).ground_type;
        assert_eq!(kind(0.0, 0.0), Normal);
        assert_eq!(kind(0.24, 0.49), Normal);
        assert_eq!(kind(0.26, 0.0), Rock);
        assert_eq!(kind(1.0, 0.0), Sand);
        assert_eq!(kind(0.0, 0.51), Mud);
        assert_eq!(kind(1.0, 1.0), Rock);
        // Outside the footprint clamps to the edge.
        assert_eq!(kind(-3.0, 9.0), Mud);
        assert_eq!(kind(f64::NAN, 0.0), Normal);
        assert!((map.max_sinkage_m() - 0.12).abs() < 1e-12);
    }

    #[test]
    fn rejects_malformed_maps() {
        let normal = [GroundMaterial::default()];
        assert!(matches!(
            GroundTypeMap::new(0, 1, vec![], normal.to_vec()),
            Err(GroundTypeMapError::EmptyGrid { .. })
        ));
        assert!(matches!(
            GroundTypeMap::new(2, 2, vec![0; 3], normal.to_vec()),
            Err(GroundTypeMapError::SizeMismatch { .. })
        ));
        assert!(matches!(
            GroundTypeMap::new(1, 1, vec![1], normal.to_vec()),
            Err(GroundTypeMapError::IndexOutOfRange { .. })
        ));
        assert!(matches!(
            GroundTypeMap::new(1, 1, vec![0], Vec::new()),
            Err(GroundTypeMapError::MaterialCount(0))
        ));
        let bad = GroundMaterial {
            friction: -1.0,
            ..GroundMaterial::default()
        };
        assert!(matches!(
            GroundTypeMap::new(1, 1, vec![0], vec![bad]),
            Err(GroundTypeMapError::InvalidMaterial(0))
        ));
    }

    #[test]
    fn decodes_painted_png_with_nearest_colours() {
        let legend = GroundTypeMap::default_legend();
        let mut rgba = Vec::new();
        for kind in GroundType::ALL {
            let [r, g, b] = kind.map_color_rgb();
            // Slightly off colours, as an anti-aliased brush leaves them.
            rgba.extend_from_slice(&[r.saturating_add(9), g.saturating_sub(7), b, 255]);
        }
        let png = encode_png(2, 2, png::ColorType::Rgba, &rgba);
        let map = GroundTypeMap::from_png(&png, &legend).unwrap();
        assert_eq!((map.columns(), map.rows()), (2, 2));
        let kinds: Vec<_> = map
            .cells()
            .iter()
            .map(|&index| map.materials()[usize::from(index)].ground_type)
            .collect();
        assert_eq!(kinds, GroundType::ALL);

        // Greyscale images match the grey (rock) entry or the nearest colour.
        let grey = encode_png(2, 1, png::ColorType::Grayscale, &[0x80, 0x82]);
        let map = GroundTypeMap::from_png(&grey, &legend).unwrap();
        assert!(map
            .cells()
            .iter()
            .all(|&index| map.materials()[usize::from(index)].ground_type == GroundType::Rock));

        assert!(matches!(
            GroundTypeMap::from_png(b"not a png", &legend),
            Err(GroundTypeMapError::Image(_))
        ));
        assert_eq!(
            GroundTypeMap::from_png(&png, &[]),
            Err(GroundTypeMapError::EmptyLegend)
        );
    }

    #[test]
    fn serde_round_trip() {
        let map =
            GroundTypeMap::from_ground_types(2, 1, &[GroundType::Sand, GroundType::Rock]).unwrap();
        let json = serde_json::to_string(&map).unwrap();
        let back: GroundTypeMap = serde_json::from_str(&json).unwrap();
        assert_eq!(back, map);
        assert!(back.is_valid());
    }

    #[test]
    fn rgb_encoding_round_trips_presets() {
        let encoding = GroundMapEncoding::default();
        assert_eq!(
            encoding.encode(&GroundMaterial::preset(GroundType::Sand)),
            [170, 64, 0x80]
        );
        assert_eq!(
            encoding.encode(&GroundMaterial::preset(GroundType::Mud)),
            [255, 38, 0x88]
        );
        for kind in GroundType::ALL {
            let preset = GroundMaterial::preset(kind);
            let decoded = encoding.decode(encoding.encode(&preset));
            assert_eq!(decoded.ground_type, kind);
            assert!((decoded.friction - preset.friction).abs() <= 1.0 / 255.0);
            assert_eq!(decoded.restitution, preset.restitution);
            match (decoded.deformation, preset.deformation) {
                (Some(soil), Some(nominal)) => {
                    assert_eq!(
                        soil.frictional_modulus_n_per_m_n2,
                        nominal.frictional_modulus_n_per_m_n2
                    );
                    assert!((soil.creep_velocity_m_s - nominal.creep_velocity_m_s).abs() < 1e-12);
                    let ratio = decoded.max_friction / decoded.friction;
                    assert!((ratio - preset.max_friction / preset.friction).abs() < 1e-5);
                }
                (None, None) => assert!(decoded.max_friction.is_infinite()),
                other => panic!("{kind:?}: {other:?}"),
            }
        }
    }

    #[test]
    fn rgb_encoding_channels() {
        let encoding = GroundMapEncoding::default();
        // R snaps to the nearest type level.
        let kinds: Vec<_> = [0u8, 42, 43, 127, 128, 212, 213, 255]
            .into_iter()
            .map(GroundMapEncoding::decode_type)
            .collect();
        use GroundType::*;
        assert_eq!(kinds, [Normal, Normal, Rock, Rock, Sand, Sand, Mud, Mud]);
        // G spans the friction range.
        assert_eq!(encoding.decode([0, 0, 0]).friction, 0.0);
        assert_eq!(encoding.decode([85, 255, 0]).friction, 2.0);
        // B: deformability scales stiffness by 2^((8 - d) / 2); creep steps.
        let nominal = GroundMaterial::preset(Sand).deformation.unwrap();
        let stiff = encoding.decode([170, 64, 0x60]).deformation.unwrap();
        let soft = encoding.decode([170, 64, 0xC5]).deformation.unwrap();
        let k = |soil: crate::SoilDeformation| soil.frictional_modulus_n_per_m_n2;
        assert!((k(stiff) / k(nominal) - 2.0).abs() < 1e-12);
        assert!((k(soft) / k(nominal) - 0.25).abs() < 1e-12);
        assert!((soft.creep_velocity_m_s - 5.0 * 0.00375).abs() < 1e-12);
        assert_eq!(stiff.creep_velocity_m_s, 0.0);
        // Rigid kinds ignore B.
        assert!(encoding.decode([85, 100, 0xFF]).deformation.is_none());
        assert!(encoding.decode([255, 38, 0x8F]).is_valid());
    }

    #[test]
    fn decodes_encoded_png_and_shares_materials() {
        let encoding = GroundMapEncoding::default();
        let sand = encoding.encode(&GroundMaterial::preset(GroundType::Sand));
        let mud = encoding.encode(&GroundMaterial::preset(GroundType::Mud));
        // R 160 is still sand, so it shares the sand entry.
        let rgb = [sand, mud, [160, sand[1], sand[2]], [0, 64, 0]].concat();
        let png = encode_png(2, 2, png::ColorType::Rgb, &rgb);
        let map = GroundTypeMap::from_encoded_png(&png, &encoding).unwrap();
        assert_eq!(map.materials().len(), 3);
        assert_eq!(map.cells(), &[0, 1, 0, 2]);
        assert_eq!(map.material_at(0.0, 0.0).ground_type, GroundType::Sand);
        assert_eq!(map.material_at(1.0, 0.0).ground_type, GroundType::Mud);
        assert_eq!(map.material_at(1.0, 1.0).ground_type, GroundType::Normal);

        let grey = encode_png(1, 1, png::ColorType::Grayscale, &[0]);
        assert!(matches!(
            GroundTypeMap::from_encoded_png(&grey, &encoding),
            Err(GroundTypeMapError::Image(_))
        ));
    }
}
