use ncollide3d::bounding_volume::HasBoundingVolume;
use ncollide3d::bounding_volume::AABB;
use ncollide3d::na;
use ncollide3d::query::{Ray, RayCast};
use ncollide3d::shape::Cylinder;
use rand::Rng;
use rand_distr::{Distribution, Normal};
use resolve_collision::lidar::{create_rays, LidarInfo};
use resolve_collision::pcd_io::write_pcd;
use std::f64::consts::PI;

use clap::Parser;
use serde::{Deserialize, Serialize};

#[derive(Parser, Debug)]
struct Cli {
    #[arg(long)]
    options: String,

    #[arg(long)]
    output_dir: String,
}

#[derive(Serialize, Deserialize, Debug)]
struct Options {
    num: i64,
    diameter_range: (f64, f64),
    r_range: (f64, f64),
    theta_range: (f64, f64),
    height_range: (f64, f64),
    lidar_infos: Vec<LidarInfo>,
}

#[derive(Serialize, Deserialize, Debug)]
struct GroundTruth {
    center: (f64, f64, f64),
    diameter: f64,
    is_complete: bool,
}

fn main() {
    let args = Cli::parse();
    let options: Options =
        serde_json::from_reader(std::fs::File::open(&args.options).unwrap()).unwrap();
    let rays = options
        .lidar_infos
        .iter()
        .map(|info| create_rays(info))
        .flatten()
        .collect::<Vec<Ray<f64>>>();
    let mut rng = rand::thread_rng();
    let normal = Normal::new(0.0, 0.005).unwrap();
    for i in 0..options.num {
        println!("{} / {}", i, options.num);
        let diameter = rng.gen_range(options.diameter_range.0..options.diameter_range.1);
        let circle = Cylinder::new(0.0015, diameter / 2.0);
        let circle_tf = na::Isometry3::from_parts(
            na::Translation3::new(0.0, 0.0, 0.0),
            na::UnitQuaternion::from_euler_angles(PI / 2.0, PI / 2.0, 0.0),
        );
        let mut circle_points = vec![];
        for i in 0..24 {
            let theta = 2.0 * PI * (i as f64) / 24.0;
            let x = diameter / 2.0 * theta.cos();
            let z = diameter / 2.0 * theta.sin();
            circle_points.push(na::Point3::new(x, 0.0, z));
        }

        let r = rng.gen_range(options.r_range.0..options.r_range.1);
        let theta = rng
            .gen_range(options.theta_range.0..options.theta_range.1)
            .to_radians();
        let x = r * theta.cos();
        let y = r * theta.sin();
        let z = rng.gen_range(options.height_range.0..options.height_range.1);
        let circle_tf = na::Isometry3::from_parts(
            na::Translation3::new(x, y, z),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, theta),
        ) * circle_tf;
        let circle_points = circle_points
            .iter()
            .map(|p| circle_tf * p)
            .collect::<Vec<na::Point3<f64>>>();

        let circle_aabb: AABB<f64> = circle.bounding_volume(&circle_tf);
        let mut points = vec![];
        const MAX_TOI: f64 = 10.0;
        for ray in rays.iter() {
            if circle_aabb.intersects_ray(&na::Isometry3::identity(), ray, MAX_TOI) {
                let toi = circle.toi_with_ray(&circle_tf, ray, MAX_TOI, true);
                if let Some(toi) = toi {
                    let toi_with_noise = toi + normal.sample(&mut rng);
                    let point = ray.point_at(toi_with_noise);
                    points.push(point);
                }
            }
        }

        let mut is_complete = true;
        for cp in circle_points.iter() {
            let mut min_dist = std::f64::MAX;
            for p in points.iter() {
                min_dist = min_dist.min((cp - p).norm());
            }
            if min_dist > 0.01 {
                is_complete = false;
                break;
            }
        }

        let gt = GroundTruth {
            center: (x, y, z),
            diameter,
            is_complete,
        };
        let _ = serde_json::to_writer(
            std::fs::File::create(format!("{}/{:04}.json", args.output_dir, i)).unwrap(),
            &gt,
        );
        write_pcd(
            &points,
            format!("{}/{:04}.pcd", args.output_dir, i).as_str(),
        );
    }
}
