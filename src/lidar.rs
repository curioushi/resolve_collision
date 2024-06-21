use crate::common::Isometry3Serde;
use ncollide3d::na;
use ncollide3d::query::Ray;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct LidarInfo {
    pub tf: Isometry3Serde,
    pub model: String,
    pub num_frames: Option<usize>,
}

pub fn create_rays(lidar_info: &LidarInfo) -> Vec<Ray<f64>> {
    let mut rays: Vec<Ray<f64>> = vec![];
    if lidar_info.model == "LIVOX-MID-360" {
        let points_per_second = 200000;
        let num_frames = lidar_info.num_frames.clone().unwrap();
        let num_points = num_frames * 20000;
        for i in 0..num_points {
            let t = i as f64 / points_per_second as f64;
            let vertical_angle = 0.45 * (62.055 * t).sin() + 0.076 * (1000000.0 * t).sin() + 0.4;
            let horizontal_angle =
                (1137.0 * t).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI;
            let origin = na::Point3::new(0.0, 0.0, 0.0);
            let dir = na::Vector3::new(
                vertical_angle.cos() * horizontal_angle.cos(),
                vertical_angle.cos() * horizontal_angle.sin(),
                vertical_angle.sin(),
            );
            rays.push(Ray::new(origin, dir));
        }
    }
    for ray in rays.iter_mut() {
        ray.origin = lidar_info.tf.transform_point(&ray.origin);
        ray.dir = lidar_info.tf.transform_vector(&ray.dir);
    }
    rays
}
