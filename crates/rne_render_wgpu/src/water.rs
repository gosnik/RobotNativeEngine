//! Optional translucent water: a flat free surface drawn after the opaque
//! scene, depth-tested against it without writing depth, and visible from
//! above and below. With the camera below the surface a full-screen tint
//! gives a mild underwater look.

use rne_math::{Mat4, Vec3};

const WATER_SHADER: &str = r#"
struct WaterUniform {
    view_proj: mat4x4<f32>,
    // Surface extent: min x, min z, max x, max z.
    rect: vec4<f32>,
    // Surface height, camera height.
    level: vec4<f32>,
    color: vec4<f32>,
    tint: vec4<f32>,
}

@group(0) @binding(0) var<uniform> water: WaterUniform;

struct SurfaceOut {
    @builtin(position) position: vec4<f32>,
    @location(0) world: vec3<f32>,
}

@vertex
fn vs_surface(@builtin(vertex_index) index: u32) -> SurfaceOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 0.0), vec2<f32>(1.0, 1.0),
        vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 1.0), vec2<f32>(0.0, 1.0),
    );
    let xz = mix(water.rect.xy, water.rect.zw, corners[index]);
    let world = vec3<f32>(xz.x, water.level.x, xz.y);
    var out: SurfaceOut;
    out.position = water.view_proj * vec4<f32>(world, 1.0);
    out.world = world;
    return out;
}

@fragment
fn fs_surface(in: SurfaceOut) -> @location(0) vec4<f32> {
    // Gentle static ripple shading so the surface reads as water.
    let ripple = sin(in.world.x * 7.0 + in.world.z * 3.0) * sin(in.world.z * 6.0 - in.world.x * 2.5);
    let shade = 0.92 + 0.12 * ripple;
    // Seen from below, the surface is brighter and more transparent.
    let below = water.level.y < water.level.x;
    let alpha = select(water.color.a, water.color.a * 0.6, below);
    let rgb = select(water.color.rgb * shade, min(water.color.rgb * 1.4 * shade, vec3<f32>(1.0)), below);
    return vec4<f32>(rgb, alpha);
}

@vertex
fn vs_tint(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    return vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
}

@fragment
fn fs_tint() -> @location(0) vec4<f32> {
    return water.tint;
}
"#;

/// A flat water surface at `level_y_m` (world +Y up) over a rectangle in X/Z.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WaterSurface {
    /// Height of the free surface in metres.
    pub level_y_m: f32,
    /// Minimum world X and Z of the drawn surface.
    pub min_xz_m: [f32; 2],
    /// Maximum world X and Z of the drawn surface.
    pub max_xz_m: [f32; 2],
    /// Linear RGB and opacity of the surface.
    pub color_rgba: [f32; 4],
    /// Linear RGB and opacity of the full-screen tint with the camera below
    /// the surface; zero opacity disables it.
    pub underwater_tint_rgba: [f32; 4],
}

impl WaterSurface {
    fn valid(&self) -> bool {
        self.level_y_m.is_finite()
            && self.min_xz_m.iter().chain(&self.max_xz_m).all(|v| v.is_finite())
            && self.min_xz_m[0] < self.max_xz_m[0]
            && self.min_xz_m[1] < self.max_xz_m[1]
            && self
                .color_rgba
                .iter()
                .chain(&self.underwater_tint_rgba)
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct WaterUniform {
    view_proj: [[f32; 4]; 4],
    rect: [f32; 4],
    level: [f32; 4],
    color: [f32; 4],
    tint: [f32; 4],
}

pub(crate) struct WaterRenderer {
    surface_pipeline: wgpu::RenderPipeline,
    tint_pipeline: wgpu::RenderPipeline,
    buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

impl WaterRenderer {
    pub(crate) fn new(device: &wgpu::Device, color_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rne_water_shader"),
            source: wgpu::ShaderSource::Wgsl(WATER_SHADER.into()),
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("rne_water_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_water_uniform"),
            size: std::mem::size_of::<WaterUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_water_bind_group"),
            layout: &layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: buffer.as_entire_binding(),
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("rne_water_pipeline_layout"),
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });
        let pipeline = |label, vs, fs, depth_compare| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some(vs),
                    buffers: &[],
                    compilation_options: Default::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(fs),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: color_format,
                        blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: Default::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    cull_mode: None,
                    ..Default::default()
                },
                // Reads the opaque scene's depth; never writes it, so the
                // surface hides what lies beyond it only by blending.
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: false,
                    depth_compare,
                    stencil: wgpu::StencilState::default(),
                    bias: wgpu::DepthBiasState::default(),
                }),
                multisample: wgpu::MultisampleState::default(),
                multiview: None,
                cache: None,
            })
        };
        Self {
            surface_pipeline: pipeline(
                "rne_water_surface",
                "vs_surface",
                "fs_surface",
                wgpu::CompareFunction::Less,
            ),
            tint_pipeline: pipeline(
                "rne_water_tint",
                "vs_tint",
                "fs_tint",
                wgpu::CompareFunction::Always,
            ),
            buffer,
            bind_group,
        }
    }

    /// Draws the surface (and, below it, the tint) over the finished scene.
    pub(crate) fn encode(
        &self,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        color_view: &wgpu::TextureView,
        depth_view: &wgpu::TextureView,
        view_proj: Mat4,
        camera_position: Vec3,
        water: &WaterSurface,
    ) {
        if !water.valid() {
            return;
        }
        let uniform = WaterUniform {
            view_proj: crate::primitive::mat4_to_cols(view_proj),
            rect: [
                water.min_xz_m[0],
                water.min_xz_m[1],
                water.max_xz_m[0],
                water.max_xz_m[1],
            ],
            level: [water.level_y_m, camera_position.y as f32, 0.0, 0.0],
            color: water.color_rgba,
            tint: water.underwater_tint_rgba,
        };
        queue.write_buffer(&self.buffer, 0, bytemuck::bytes_of(&uniform));
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("rne_water_pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: color_view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        pass.set_bind_group(0, &self.bind_group, &[]);
        pass.set_pipeline(&self.surface_pipeline);
        pass.draw(0..6, 0..1);
        if (camera_position.y as f32) < water.level_y_m && water.underwater_tint_rgba[3] > 0.0 {
            pass.set_pipeline(&self.tint_pipeline);
            pass.draw(0..3, 0..1);
        }
    }
}
