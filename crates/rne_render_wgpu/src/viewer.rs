//! Interactive winit window backed by wgpu.

use crate::overlay::{DepthOverlay, DepthOverlayDraw, ImageOverlay, ImageOverlayDraw};
use crate::primitive::{PrimitiveRenderViews, PrimitiveRenderer, PrimitiveSurfacePass};
use crate::taa::TaaSettings;
use rne_math::Transform3;
use rne_render::{Camera, EnvironmentLighting, RenderError, RenderScene};
use std::sync::Arc;
use thiserror::Error;
use winit::window::Window;

/// Errors while creating or presenting an interactive viewer.
#[derive(Debug, Error)]
pub enum ViewerError {
    /// No compatible GPU adapter was found.
    #[error("no compatible GPU adapter")]
    NoAdapter,
    /// GPU device initialization failed.
    #[error("GPU init failed: {0}")]
    InitFailed(String),
    /// Surface or swapchain operation failed.
    #[error("surface error: {0}")]
    Surface(String),
    /// The live surface cannot be copied to a readback buffer.
    #[error("surface does not support screenshots")]
    ScreenshotUnsupported,
    /// A requested live-frame readback failed.
    #[error("screenshot readback failed: {0}")]
    Screenshot(String),
}

impl From<RenderError> for ViewerError {
    fn from(error: RenderError) -> Self {
        match error {
            RenderError::NoAdapter => Self::NoAdapter,
            RenderError::InitFailed(message) => Self::InitFailed(message),
            other => Self::Surface(other.to_string()),
        }
    }
}

/// winit window with a wgpu swapchain for interactive scene rendering.
pub struct InteractiveViewer {
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    primitive: PrimitiveRenderer,
    environment: EnvironmentLighting,
    taa: TaaSettings,
    depth_texture: wgpu::Texture,
    depth_view: wgpu::TextureView,
    overlay: ImageOverlay,
    depth_overlay: DepthOverlay,
    screenshot_supported: bool,
    screenshot_requested: bool,
    screenshot: Option<ViewerScreenshot>,
}

/// Pixels copied from the actual composited presentation texture.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ViewerScreenshot {
    /// Tightly packed RGBA8 pixels in presentation order.
    pub rgba8: Vec<u8>,
    /// Captured surface width in pixels.
    pub width: u32,
    /// Captured surface height in pixels.
    pub height: u32,
}

impl InteractiveViewer {
    /// Creates a viewer bound to an existing winit window.
    pub fn new(window: Arc<Window>) -> Result<Self, ViewerError> {
        pollster::block_on(Self::new_async(window))
    }

    /// Creates a viewer asynchronously (required on `wasm32`).
    pub async fn new_async(window: Arc<Window>) -> Result<Self, ViewerError> {
        let size = window.inner_size();
        let width = size.width.max(1);
        let height = size.height.max(1);

        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });
        let surface = instance
            .create_surface(window.clone())
            .map_err(|error| ViewerError::Surface(error.to_string()))?;

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .ok_or(ViewerError::NoAdapter)?;

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("rne_interactive_viewer"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    memory_hints: wgpu::MemoryHints::Performance,
                },
                None,
            )
            .await
            .map_err(|error| ViewerError::InitFailed(error.to_string()))?;

        let surface_caps = surface.get_capabilities(&adapter);
        let format = surface_caps
            .formats
            .iter()
            .copied()
            .find(wgpu::TextureFormat::is_srgb)
            .unwrap_or(surface_caps.formats[0]);

        let screenshot_supported = surface_caps.usages.contains(wgpu::TextureUsages::COPY_SRC)
            && matches!(
                format,
                wgpu::TextureFormat::Rgba8Unorm
                    | wgpu::TextureFormat::Rgba8UnormSrgb
                    | wgpu::TextureFormat::Bgra8Unorm
                    | wgpu::TextureFormat::Bgra8UnormSrgb
            );
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | screenshot_supported
                    .then_some(wgpu::TextureUsages::COPY_SRC)
                    .unwrap_or(wgpu::TextureUsages::empty()),
            format,
            width,
            height,
            present_mode: wgpu::PresentMode::Fifo,
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &config);

        let primitive = PrimitiveRenderer::new(&device, &queue, format);
        let (depth_texture, depth_view) = create_depth_target(&device, width, height);
        let overlay = ImageOverlay::new(&device, format);
        let depth_overlay = DepthOverlay::new(&device, format);

        Ok(Self {
            window,
            surface,
            device,
            queue,
            config,
            primitive,
            environment: EnvironmentLighting::default(),
            taa: TaaSettings::default(),
            depth_texture,
            depth_view,
            overlay,
            depth_overlay,
            screenshot_supported,
            screenshot_requested: false,
            screenshot: None,
        })
    }

    /// Returns the underlying winit window.
    pub fn window(&self) -> &Window {
        &self.window
    }

    /// Returns the current drawable size in pixels.
    pub fn size(&self) -> (u32, u32) {
        (self.config.width, self.config.height)
    }

    /// Requests a one-frame copy of the exact composited swapchain image.
    /// Readback blocks only while this explicit request is serviced.
    pub fn request_screenshot(&mut self) -> Result<(), ViewerError> {
        if !self.screenshot_supported {
            return Err(ViewerError::ScreenshotUnsupported);
        }
        self.screenshot_requested = true;
        Ok(())
    }

    /// Takes the most recently requested live-frame capture.
    pub fn take_screenshot(&mut self) -> Option<ViewerScreenshot> {
        self.screenshot.take()
    }

    /// Replaces the HDR environment used for subsequent frames.
    pub fn set_environment(&mut self, environment: EnvironmentLighting) {
        let environment = environment.sanitized();
        if self.environment != environment {
            self.environment = environment;
            self.reset_taa_history();
        }
    }

    /// Replaces the temporal anti-aliasing settings for subsequent frames.
    pub fn set_taa(&mut self, settings: TaaSettings) {
        self.taa = settings.sanitized();
        self.primitive.set_taa(self.taa);
    }

    /// Returns the current temporal anti-aliasing settings.
    pub fn taa(&self) -> TaaSettings {
        self.taa
    }

    /// Discards temporal history before the next frame.
    pub fn reset_taa_history(&mut self) {
        self.primitive.reset_taa_history();
    }

    /// Builds a pinhole camera matching the window size.
    pub fn camera(&self) -> Camera {
        Camera::new(
            self.config.width,
            self.config.height,
            std::f64::consts::FRAC_PI_4,
        )
    }

    /// Resizes the swapchain and depth target.
    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }

        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.device, &self.config);

        let (depth_texture, depth_view) = create_depth_target(&self.device, width, height);
        self.depth_texture = depth_texture;
        self.depth_view = depth_view;
    }

    /// Renders a scene to the window and presents the frame.
    pub fn render(
        &mut self,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
    ) -> Result<(), ViewerError> {
        self.render_with_pip(view, scene, clear_color, None)
    }

    /// Renders a scene and optionally composites an RGBA8 PiP overlay before present.
    pub fn render_with_pip(
        &mut self,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
        pip: Option<(Vec<u8>, u32, u32)>,
    ) -> Result<(), ViewerError> {
        self.render_with_pip_and_depth(view, scene, clear_color, pip, None)
    }

    /// Renders a scene and composites an optional RGBA8 overlay over the full
    /// viewport before presenting the frame.
    pub fn render_with_overlay(
        &mut self,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
        overlay: Option<(Vec<u8>, u32, u32)>,
    ) -> Result<(), ViewerError> {
        let output = self
            .surface
            .get_current_texture()
            .map_err(|error| ViewerError::Surface(error.to_string()))?;
        let color_view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let camera = self.camera();
        self.primitive
            .render_to_views(PrimitiveSurfacePass {
                device: &self.device,
                queue: &self.queue,
                camera: &camera,
                view,
                scene,
                environment: &self.environment,
                clear_color,
                preserve_color: false,
                targets: &PrimitiveRenderViews {
                    width: self.config.width,
                    height: self.config.height,
                    color_view: &color_view,
                    depth_view: &self.depth_view,
                },
            })
            .map_err(ViewerError::from)?;
        if let Some((rgba8, width, height)) = overlay {
            let mut encoder = self
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("rne_viewer_overlay_encoder"),
                });
            self.overlay.draw(
                &self.device,
                &self.queue,
                ImageOverlayDraw {
                    encoder: &mut encoder,
                    color_view: &color_view,
                    rgba8: &rgba8,
                    width,
                    height,
                    ndc_min: [-1.0, -1.0],
                    ndc_max: [1.0, 1.0],
                },
            );
            self.queue.submit(std::iter::once(encoder.finish()));
        }
        self.capture_requested_frame(&output.texture)?;
        output.present();
        Ok(())
    }

    /// Returns the graphics resources for a UI renderer sharing this surface.
    pub fn graphics_context(&self) -> (&wgpu::Device, &wgpu::Queue, wgpu::TextureFormat) {
        (&self.device, &self.queue, self.config.format)
    }

    /// Renders a scene, then runs a GPU overlay pass before capture and present.
    /// The overlay must preserve the scene color. Captures include the overlay.
    pub fn render_with_gui(
        &mut self,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
        draw: impl FnOnce(&wgpu::Device, &wgpu::Queue, &wgpu::TextureView),
    ) -> Result<(), ViewerError> {
        let output = self
            .surface
            .get_current_texture()
            .map_err(|error| ViewerError::Surface(error.to_string()))?;
        let color_view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let camera = self.camera();
        self.primitive
            .render_to_views(PrimitiveSurfacePass {
                device: &self.device,
                queue: &self.queue,
                camera: &camera,
                view,
                scene,
                environment: &self.environment,
                clear_color,
                preserve_color: false,
                targets: &PrimitiveRenderViews {
                    width: self.config.width,
                    height: self.config.height,
                    color_view: &color_view,
                    depth_view: &self.depth_view,
                },
            })
            .map_err(ViewerError::from)?;
        draw(&self.device, &self.queue, &color_view);
        self.capture_requested_frame(&output.texture)?;
        output.present();
        Ok(())
    }

    /// Renders a scene with optional RGBA8 and GPU depth PiP overlays.
    pub fn render_with_pip_and_depth(
        &mut self,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
        pip: Option<(Vec<u8>, u32, u32)>,
        depth_pip: Option<(Vec<f32>, u32, u32)>,
    ) -> Result<(), ViewerError> {
        let output = self
            .surface
            .get_current_texture()
            .map_err(|error| ViewerError::Surface(error.to_string()))?;
        let color_view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let camera = self.camera();
        self.primitive
            .render_to_views(PrimitiveSurfacePass {
                device: &self.device,
                queue: &self.queue,
                camera: &camera,
                view,
                scene,
                environment: &self.environment,
                clear_color,
                preserve_color: false,
                targets: &PrimitiveRenderViews {
                    width: self.config.width,
                    height: self.config.height,
                    color_view: &color_view,
                    depth_view: &self.depth_view,
                },
            })
            .map_err(ViewerError::from)?;

        if let Some((rgba8, width, height)) = pip {
            let scale = 0.28_f32;
            let aspect = height as f32 / width as f32;
            let pip_w = scale;
            let pip_h = scale * aspect * (self.config.width as f32 / self.config.height as f32);
            let margin = 0.02;
            let x0 = 1.0 - pip_w - margin;
            let y0 = -1.0 + margin;
            let x1 = 1.0 - margin;
            let y1 = y0 + pip_h * 2.0;

            let mut encoder = self
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("rne_viewer_pip_encoder"),
                });
            self.overlay.draw(
                &self.device,
                &self.queue,
                ImageOverlayDraw {
                    encoder: &mut encoder,
                    color_view: &color_view,
                    rgba8: &rgba8,
                    width,
                    height,
                    ndc_min: [x0, y0],
                    ndc_max: [x1, y1],
                },
            );
            self.queue.submit(std::iter::once(encoder.finish()));
        }

        if let Some((depth_f32, width, height)) = depth_pip {
            let max_depth_m = depth_f32
                .iter()
                .copied()
                .filter(|depth| depth.is_finite() && *depth > 0.0)
                .fold(0.0_f32, f32::max)
                .max(1.0);
            let scale = 0.28_f32;
            let aspect = height as f32 / width as f32;
            let pip_w = scale;
            let pip_h = scale * aspect * (self.config.width as f32 / self.config.height as f32);
            let margin = 0.02;
            let x0 = -1.0 + margin;
            let y0 = -1.0 + margin;
            let x1 = x0 + pip_w;
            let y1 = y0 + pip_h * 2.0;

            let mut encoder = self
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("rne_viewer_depth_pip_encoder"),
                });
            self.depth_overlay.draw(
                &self.device,
                &self.queue,
                DepthOverlayDraw {
                    encoder: &mut encoder,
                    color_view: &color_view,
                    depth_f32: &depth_f32,
                    width,
                    height,
                    max_depth_m,
                    ndc_min: [x0, y0],
                    ndc_max: [x1, y1],
                },
            );
            self.queue.submit(std::iter::once(encoder.finish()));
        }

        output.present();
        Ok(())
    }
}

impl InteractiveViewer {
    fn capture_requested_frame(&mut self, texture: &wgpu::Texture) -> Result<(), ViewerError> {
        if !self.screenshot_requested {
            return Ok(());
        }
        self.screenshot_requested = false;
        let width = self.config.width;
        let height = self.config.height;
        let packed_row = width
            .checked_mul(4)
            .ok_or_else(|| ViewerError::Screenshot("row size overflow".into()))?;
        let bytes_per_row = align_to(packed_row, wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
        let buffer_size = u64::from(bytes_per_row)
            .checked_mul(u64::from(height))
            .ok_or_else(|| ViewerError::Screenshot("buffer size overflow".into()))?;
        let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_viewer_screenshot_readback"),
            size: buffer_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("rne_viewer_screenshot_encoder"),
            });
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(bytes_per_row),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        self.queue.submit(std::iter::once(encoder.finish()));
        let slice = buffer.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = sender.send(result);
        });
        self.device.poll(wgpu::Maintain::Wait);
        receiver
            .recv()
            .map_err(|_| ViewerError::Screenshot("readback channel closed".into()))?
            .map_err(|error| ViewerError::Screenshot(error.to_string()))?;
        let mapped = slice.get_mapped_range();
        let mut rgba8 = copy_padded_rgba8(&mapped, width, height, bytes_per_row);
        drop(mapped);
        buffer.unmap();
        if matches!(
            self.config.format,
            wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb
        ) {
            for pixel in rgba8.chunks_exact_mut(4) {
                pixel.swap(0, 2);
            }
        }
        self.screenshot = Some(ViewerScreenshot {
            rgba8,
            width,
            height,
        });
        Ok(())
    }
}

fn align_to(value: u32, alignment: u32) -> u32 {
    value.div_ceil(alignment) * alignment
}

fn copy_padded_rgba8(bytes: &[u8], width: u32, height: u32, bytes_per_row: u32) -> Vec<u8> {
    let packed_row = width as usize * 4;
    let mut rgba8 = vec![0; packed_row * height as usize];
    for row in 0..height as usize {
        let source = row * bytes_per_row as usize;
        let destination = row * packed_row;
        rgba8[destination..destination + packed_row]
            .copy_from_slice(&bytes[source..source + packed_row]);
    }
    rgba8
}

fn create_depth_target(
    device: &wgpu::Device,
    width: u32,
    height: u32,
) -> (wgpu::Texture, wgpu::TextureView) {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("rne_viewer_depth"),
        size: wgpu::Extent3d {
            width: width.max(1),
            height: height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    (texture, view)
}

#[cfg(test)]
mod tests {
    use super::copy_padded_rgba8;
    use crate::camera::CameraOrbit;
    use rne_math::Vec3;

    #[test]
    fn camera_orbit_produces_finite_transform() {
        let orbit = CameraOrbit {
            yaw_rad: 0.7,
            pitch_rad: 0.4,
            distance_m: 3.0,
            focus: Vec3::new(1.0, 0.25, 0.0),
        };
        let transform = orbit.camera_transform();
        assert!(transform.translation.x.is_finite());
        assert!(transform.translation.y.is_finite());
        assert!(transform.translation.z.is_finite());
    }

    #[test]
    fn screenshot_rows_strip_wgpu_padding() {
        let source = [
            1, 2, 3, 4, 5, 6, 7, 8, 99, 99, 99, 99, 9, 10, 11, 12, 13, 14, 15, 16, 99, 99, 99, 99,
        ];
        assert_eq!(
            copy_padded_rgba8(&source, 2, 2, 12),
            vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        );
    }
}
