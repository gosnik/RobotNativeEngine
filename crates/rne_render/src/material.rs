//! Backend-neutral physically based material parameters.

use crate::image::ImageFrame;
use std::sync::Arc;

/// Optional world-projected micro-detail for broad, mostly horizontal surfaces.
///
/// The renderer samples `image` in world XZ coordinates.  It is a visual-only
/// layer: it never changes mesh geometry, collision, or material physics.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceDetail {
    /// Repeatable RGB detail image. Its luminance supplies color and bump detail.
    pub image: Arc<ImageFrame>,
    /// Image repeats in each world-space metre.
    pub repeats_per_metre: f32,
    /// Blend strength of image luminance into the existing albedo.
    pub albedo_strength: f32,
    /// Strength of the subtle world-space bump derived from image luminance.
    pub normal_strength: f32,
    /// Strength of broad, continuous seeded color variation.
    pub color_variation: f32,
    /// Crossfade rotated image and base-color samples to reduce visible
    /// repetition on flat terrain. This intentionally breaks mesh-UV alignment,
    /// so use it only where that alignment is not authored detail.
    pub anti_tiling: bool,
}

impl SurfaceDetail {
    /// Creates a conservatively tuned surface-detail layer.
    pub fn new(image: Arc<ImageFrame>) -> Self {
        Self {
            image,
            repeats_per_metre: 1.0,
            albedo_strength: 0.35,
            normal_strength: 0.35,
            color_variation: 0.2,
            anti_tiling: false,
        }
    }

    /// Returns parameters safe for GPU upload while retaining the source image.
    pub fn sanitized(&self) -> Self {
        Self {
            image: Arc::clone(&self.image),
            repeats_per_metre: finite_or(self.repeats_per_metre, 1.0).clamp(0.01, 256.0),
            albedo_strength: finite_or(self.albedo_strength, 0.35).clamp(0.0, 1.0),
            normal_strength: finite_or(self.normal_strength, 0.35).clamp(0.0, 1.0),
            color_variation: finite_or(self.color_variation, 0.2).clamp(0.0, 1.0),
            anti_tiling: self.anti_tiling,
        }
    }

    /// Returns whether the image can be uploaded as a complete RGBA8 texture.
    pub fn has_valid_image(&self) -> bool {
        self.image.width > 0
            && self.image.height > 0
            && self
                .image
                .width
                .checked_mul(self.image.height)
                .and_then(|pixels| pixels.checked_mul(4))
                .is_some_and(|expected| expected as usize == self.image.rgba8.len())
    }
}

/// Material parameters shared by render backends.
///
/// The values describe a metallic-roughness workflow. `base_color_rgba` is
/// linear-space albedo and opacity; textures are sampled separately by the
/// backend and multiplied by this value. Backends should use [`Self::sanitized`]
/// before uploading the parameters to a GPU.
#[derive(Clone, Debug, PartialEq)]
pub struct PbrMaterial {
    /// Linear-space base color and opacity.
    pub base_color_rgba: [f32; 4],
    /// Perceptual surface roughness in the inclusive `[0, 1]` range.
    pub roughness: f32,
    /// Metallic factor in the inclusive `[0, 1]` range.
    pub metallic: f32,
    /// Linear-space emissive RGB contribution.
    pub emissive_rgb: [f32; 3],
    /// Optional tangent-space normal map sampled with mesh UV coordinates.
    pub normal_texture: Option<Arc<ImageFrame>>,
    /// Optional linear roughness map sampled with mesh UV coordinates.
    pub roughness_texture: Option<Arc<ImageFrame>>,
    /// Strength applied to tangent-space normal-map XY components.
    pub normal_strength: f32,
    /// Optional packed glTF metallic-roughness texture. Green is roughness;
    /// blue is metallic, both in linear space.
    pub metallic_roughness_texture: Option<Arc<ImageFrame>>,
    /// Optional sRGB glTF emissive texture.
    pub emissive_texture: Option<Arc<ImageFrame>>,
    /// Optional linear glTF occlusion texture sampled from the red channel.
    pub occlusion_texture: Option<Arc<ImageFrame>>,
    /// Strength applied to the occlusion texture in the inclusive `[0, 1]`
    /// range.
    pub occlusion_strength: f32,
    /// Optional world-projected visual micro-detail layer.
    pub surface_detail: Option<SurfaceDetail>,
}

impl Default for PbrMaterial {
    fn default() -> Self {
        Self {
            base_color_rgba: [1.0; 4],
            roughness: 0.7,
            metallic: 0.0,
            emissive_rgb: [0.0; 3],
            normal_texture: None,
            roughness_texture: None,
            normal_strength: 1.0,
            metallic_roughness_texture: None,
            emissive_texture: None,
            occlusion_texture: None,
            occlusion_strength: 1.0,
            surface_detail: None,
        }
    }
}

impl PbrMaterial {
    /// Creates a metallic-roughness material with explicit parameters.
    pub const fn new(
        base_color_rgba: [f32; 4],
        roughness: f32,
        metallic: f32,
        emissive_rgb: [f32; 3],
    ) -> Self {
        Self {
            base_color_rgba,
            roughness,
            metallic,
            emissive_rgb,
            normal_texture: None,
            roughness_texture: None,
            normal_strength: 1.0,
            metallic_roughness_texture: None,
            emissive_texture: None,
            occlusion_texture: None,
            occlusion_strength: 1.0,
            surface_detail: None,
        }
    }

    /// Attaches optional tangent-space normal and linear roughness textures.
    pub fn with_texture_maps(
        mut self,
        normal_texture: Option<Arc<ImageFrame>>,
        roughness_texture: Option<Arc<ImageFrame>>,
    ) -> Self {
        self.normal_texture = normal_texture;
        self.roughness_texture = roughness_texture;
        self
    }

    /// Attaches packed metallic-roughness, emissive, and occlusion maps.
    pub fn with_pbr_texture_maps(
        mut self,
        metallic_roughness_texture: Option<Arc<ImageFrame>>,
        emissive_texture: Option<Arc<ImageFrame>>,
        occlusion_texture: Option<Arc<ImageFrame>>,
        occlusion_strength: f32,
    ) -> Self {
        self.metallic_roughness_texture = metallic_roughness_texture;
        self.emissive_texture = emissive_texture;
        self.occlusion_texture = occlusion_texture;
        self.occlusion_strength = occlusion_strength;
        self
    }

    /// Sets the tangent-space normal-map scale used by a backend.
    pub fn with_normal_strength(mut self, normal_strength: f32) -> Self {
        self.normal_strength = normal_strength;
        self
    }

    /// Adds a world-projected visual detail layer.
    pub fn with_surface_detail(mut self, surface_detail: SurfaceDetail) -> Self {
        self.surface_detail = Some(surface_detail);
        self
    }

    /// Returns finite, shader-safe material parameters.
    pub fn sanitized(&self) -> Self {
        Self {
            base_color_rgba: self
                .base_color_rgba
                .map(|value| finite_or(value, 1.0).max(0.0)),
            roughness: finite_or(self.roughness, 0.7).clamp(0.04, 1.0),
            metallic: finite_or(self.metallic, 0.0).clamp(0.0, 1.0),
            emissive_rgb: self
                .emissive_rgb
                .map(|value| finite_or(value, 0.0).max(0.0)),
            normal_texture: self.normal_texture.clone(),
            roughness_texture: self.roughness_texture.clone(),
            normal_strength: finite_or(self.normal_strength, 1.0).clamp(0.0, 2.0),
            metallic_roughness_texture: self.metallic_roughness_texture.clone(),
            emissive_texture: self.emissive_texture.clone(),
            occlusion_texture: self.occlusion_texture.clone(),
            occlusion_strength: finite_or(self.occlusion_strength, 1.0).clamp(0.0, 1.0),
            surface_detail: self
                .surface_detail
                .as_ref()
                .filter(|detail| detail.has_valid_image())
                .map(SurfaceDetail::sanitized),
        }
    }
}

fn finite_or(value: f32, fallback: f32) -> f32 {
    if value.is_finite() {
        value
    } else {
        fallback
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_material_is_non_metallic_and_rough() {
        let material = PbrMaterial::default();
        assert_eq!(material.base_color_rgba, [1.0; 4]);
        assert_eq!(material.roughness, 0.7);
        assert_eq!(material.metallic, 0.0);
        assert_eq!(material.emissive_rgb, [0.0; 3]);
        assert!(material.normal_texture.is_none());
        assert!(material.roughness_texture.is_none());
        assert_eq!(material.normal_strength, 1.0);
        assert!(material.metallic_roughness_texture.is_none());
        assert!(material.emissive_texture.is_none());
        assert!(material.occlusion_texture.is_none());
        assert_eq!(material.occlusion_strength, 1.0);
        assert!(material.surface_detail.is_none());
    }

    #[test]
    fn sanitization_clamps_invalid_shader_inputs() {
        let material = PbrMaterial::new(
            [f32::NAN, -1.0, 0.5, 1.0],
            f32::INFINITY,
            -2.0,
            [f32::NAN, -1.0, 2.0],
        )
        .sanitized();
        assert_eq!(material.base_color_rgba, [1.0, 0.0, 0.5, 1.0]);
        assert_eq!(material.roughness, 0.7);
        assert_eq!(material.metallic, 0.0);
        assert_eq!(material.emissive_rgb, [0.0, 0.0, 2.0]);
        assert!(material.normal_texture.is_none());
        assert!(material.roughness_texture.is_none());
        assert_eq!(material.normal_strength, 1.0);
        assert!(material.metallic_roughness_texture.is_none());
        assert!(material.emissive_texture.is_none());
        assert!(material.occlusion_texture.is_none());
        assert_eq!(material.occlusion_strength, 1.0);
        assert!(material.surface_detail.is_none());
    }

    #[test]
    fn texture_maps_are_preserved_through_sanitization() {
        let normal = Arc::new(ImageFrame::from_rgba8(1, 1, vec![128, 128, 255, 255]));
        let roughness = Arc::new(ImageFrame::from_rgba8(1, 1, vec![180, 180, 180, 255]));
        let metallic_roughness = Arc::new(ImageFrame::from_rgba8(1, 1, vec![0, 128, 200, 255]));
        let emissive = Arc::new(ImageFrame::from_rgba8(1, 1, vec![12, 24, 48, 255]));
        let occlusion = Arc::new(ImageFrame::from_rgba8(1, 1, vec![160, 160, 160, 255]));
        let material = PbrMaterial::default()
            .with_texture_maps(Some(Arc::clone(&normal)), Some(Arc::clone(&roughness)))
            .with_pbr_texture_maps(
                Some(Arc::clone(&metallic_roughness)),
                Some(Arc::clone(&emissive)),
                Some(Arc::clone(&occlusion)),
                0.65,
            );
        let sanitized = material.sanitized();
        assert_eq!(sanitized.normal_texture, Some(normal));
        assert_eq!(sanitized.roughness_texture, Some(roughness));
        assert_eq!(
            sanitized.metallic_roughness_texture,
            Some(metallic_roughness)
        );
        assert_eq!(sanitized.emissive_texture, Some(emissive));
        assert_eq!(sanitized.occlusion_texture, Some(occlusion));
        assert_eq!(sanitized.occlusion_strength, 0.65);
    }

    #[test]
    fn surface_detail_is_preserved_and_sanitized() {
        let image = Arc::new(ImageFrame::from_rgba8(1, 1, vec![96, 128, 64, 255]));
        let material = PbrMaterial::default().with_surface_detail(SurfaceDetail {
            image: Arc::clone(&image),
            repeats_per_metre: f32::INFINITY,
            albedo_strength: -1.0,
            normal_strength: 4.0,
            color_variation: f32::NAN,
            anti_tiling: true,
        });
        let detail = material
            .sanitized()
            .surface_detail
            .expect("detail retained");
        assert_eq!(detail.image, image);
        assert_eq!(detail.repeats_per_metre, 1.0);
        assert_eq!(detail.albedo_strength, 0.0);
        assert_eq!(detail.normal_strength, 1.0);
        assert_eq!(detail.color_variation, 0.2);
        assert!(detail.anti_tiling);
    }

    #[test]
    fn malformed_surface_detail_images_are_dropped() {
        for image in [
            ImageFrame::from_rgba8(0, 1, vec![]),
            ImageFrame::from_rgba8(2, 2, vec![0; 15]),
            ImageFrame::from_rgba8(u32::MAX, u32::MAX, vec![]),
        ] {
            let material = PbrMaterial::default()
                .with_surface_detail(SurfaceDetail::new(Arc::new(image)))
                .sanitized();
            assert!(material.surface_detail.is_none());
        }
    }
}
