//! Synthetic LiDAR query-count benchmark. Run with `cargo test -p rne_sensor
//! --test lidar_benchmark -- --ignored --nocapture`; timings are workload evidence,
//! not a physics-backend benchmark.

use rne_ecs::{Entity, World};
use rne_physics::{PhysicsError, PhysicsWorldId, RaycastHit, RaycastQuery};
use rne_sensor::lidar::{
    sample_lidar_pattern_swept, LidarRay, LidarRaycaster, LidarSpec, LidarSweep,
};
use rne_sensor::SensorNoiseKey;
use rne_world::Transform3;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

struct SyntheticRaycaster {
    queries: AtomicUsize,
}

impl LidarRaycaster for SyntheticRaycaster {
    fn lidar_raycast(
        &self,
        _: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        self.queries.fetch_add(1, Ordering::Relaxed);
        Ok(vec![RaycastHit {
            entity: Entity::from_raw(0),
            point_m: query.origin_m + query.direction * 12.0,
            normal: -query.direction,
            distance_m: 12.0,
        }])
    }
}

#[test]
#[ignore = "repeatable synthetic performance workload"]
fn beam_sampling_reports_central_query_reuse() {
    let world = World::new();
    let rays: Vec<_> = (0..64)
        .map(|column| LidarRay {
            azimuth_rad: f64::from(column) * 0.01,
            elevation_rad: 0.0,
            time_s: 0.0,
            channel: 0,
            column,
        })
        .collect();
    let spec = LidarSpec {
        beam_sample_count: 16,
        beam_divergence_rad: 0.02,
        ray_count: 64,
        ..LidarSpec::default()
    };
    let backend = SyntheticRaycaster {
        queries: AtomicUsize::new(0),
    };
    let start = Instant::now();
    let cloud = sample_lidar_pattern_swept(
        &backend,
        PhysicsWorldId::DEFAULT,
        &world,
        &LidarSweep::stationary(Transform3::IDENTITY),
        &spec,
        &rays,
        SensorNoiseKey::new(7, 11, 13, 17),
    );
    let elapsed = start.elapsed();
    let queries = backend.queries.load(Ordering::Relaxed);
    let baseline_queries = rays.len() * 17;
    let improved_queries = rays.len() * 16;
    assert!(queries == baseline_queries || queries == improved_queries);
    assert_eq!(cloud.points_m.len(), 64);
    println!(
        "synthetic lidar: rays={} samples=16 queries={} saved={} elapsed={:?}",
        rays.len(),
        queries,
        baseline_queries.saturating_sub(queries),
        elapsed
    );
}
