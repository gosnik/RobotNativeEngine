//! Render a high-resolution 16-bit heightmap and its color bitmap to a PNG.
//! Usage: cargo run -p rne_render_wgpu --release --example terrain_heightmap -- HEIGHT.png COLOR.png OUTPUT.png
use rne_math::Vec3;
use rne_render::{Camera, ImageFrame, RenderBackend, RenderScene, TerrainHeightMap, TerrainMesh};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use std::{error::Error, sync::Arc};

fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 3 {
        return Err("expected HEIGHT.png COLOR.png OUTPUT.png".into());
    }
    let map = TerrainHeightMap::from_png16(&std::fs::read(&args[0])?, [6.0, 6.0], [0.0, 1.4])?;
    let tiles = TerrainMesh::new(&map, 64, 0.06)?;
    let color = image::open(&args[1])?.to_rgba8();
    let texture = Arc::new(ImageFrame::from_rgba8(
        color.width(),
        color.height(),
        color.into_raw(),
    ));
    let camera = Camera::new(1200, 900, 0.65);
    let view = CameraOrbit {
        yaw_rad: 0.7,
        pitch_rad: 0.85,
        distance_m: 12.2,
        focus: Vec3::new(0.0, 0.3, 0.0),
    }
    .camera_transform();
    let meshes = tiles.meshes_for_view(view.translation, camera.fov_y_rad, camera.height, 0.75)?;
    let triangles: usize = meshes.iter().map(|m| m.triangle_count()).sum();
    let mut scene = RenderScene::new();
    for mesh in meshes {
        let mut item = RenderScene::item_from_shared_mesh(mesh, [1.0; 4]);
        item.base_color_texture = Some(texture.clone());
        item.material.roughness = 0.95;
        scene.items.push(item);
    }
    let mut backend = WgpuRenderBackend::new()?;
    let output = backend.render_scene_camera(&camera, &view, &scene, [0.04, 0.045, 0.05, 1.0])?;
    image::save_buffer(
        &args[2],
        &output.color.rgba8,
        camera.width,
        camera.height,
        image::ColorType::Rgba8,
    )?;
    println!(
        "{} x {} heightmap; {} tiles; {triangles} selected triangles; wrote {}",
        map.columns(),
        map.rows(),
        scene.items.len(),
        args[2]
    );
    Ok(())
}
