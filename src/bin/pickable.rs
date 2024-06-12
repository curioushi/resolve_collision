use std::vec;

use clap::Parser;
use ncollide3d::bounding_volume::bounding_sphere;
use ncollide3d::bounding_volume::BoundingSphere;
use ncollide3d::na;
use ncollide3d::partitioning::{BVH, BVT};
use ncollide3d::query::visitors::RayInterferencesCollector;
use ncollide3d::query::{Ray, RayCast};
use resolve_collision::common::CuboidWithTf;

#[derive(Parser, Debug)]
struct Cli {
    #[arg(short, long)]
    input: String,

    #[arg(short, long)]
    output: String,
}

fn pickable(cubes: &Vec<CuboidWithTf>) -> Vec<bool> {
    let mut leaves: Vec<(usize, BoundingSphere<f64>)> = vec![];
    for (index, cube) in cubes.iter().enumerate() {
        leaves.push((
            index,
            bounding_sphere::bounding_sphere(cube.cuboid.as_ref(), &cube.tf),
        ));
    }
    let bvt = BVT::new_balanced(leaves.clone());
    let mut pickable_mask: Vec<bool> = vec![];
    for (i, cube) in cubes.iter().enumerate() {
        let half_extents = cube.cuboid.half_extents;
        let center_top = cube
            .tf
            .transform_point(&na::Point3::new(0.0, 0.0, half_extents[2]));
        let ray = Ray::new(center_top, na::Vector3::new(0.0, 0.0, 1.0));

        let mut interferences = Vec::new();
        let mut visitor = RayInterferencesCollector::new(&ray, f64::INFINITY, &mut interferences);
        bvt.visit(&mut visitor);

        let mut smallest_toi = f64::INFINITY;
        for j in interferences.iter() {
            if i == *j {
                continue;
            }
            let cube = &cubes[*j];
            let tf = cube.tf.as_ref();
            let toi = cube.cuboid.toi_with_ray(&tf, &ray, f64::INFINITY, true);
            if let Some(toi) = toi {
                if toi < smallest_toi {
                    smallest_toi = toi;
                }
            }
        }
        pickable_mask.push(smallest_toi == f64::INFINITY);
    }
    pickable_mask
}

fn main() {
    let start_time = std::time::Instant::now();
    let args = Cli::parse();

    // load
    let cubes = serde_json::from_str::<Vec<CuboidWithTf>>(
        &std::fs::read_to_string(&args.input).expect("Failed to read JSON file"),
    )
    .unwrap();

    // pickable
    let pickable_mask = pickable(&cubes);

    // save
    let json_data = serde_json::to_string(&pickable_mask).unwrap();
    std::fs::write(args.output, json_data).unwrap();
    let duration = start_time.elapsed();
    println!("Time elapsed: {:?}", duration);
}
