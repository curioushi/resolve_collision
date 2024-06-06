use std::vec;

use clap::Parser;
use ncollide3d::bounding_volume::bounding_sphere;
use ncollide3d::bounding_volume::BoundingSphere;
use ncollide3d::bounding_volume::AABB;
use ncollide3d::na;
use ncollide3d::partitioning::{BVH, BVT};
use ncollide3d::query::visitors::RayInterferencesCollector;
use ncollide3d::query::{Ray, RayCast};
use ncollide3d::shape::Cuboid;
use resolve_collision::common::{CubeSerde, CuboidWithTf};

#[derive(Parser, Debug)]
struct Cli {
    #[arg(short, long)]
    input_dir: String,

    #[arg(short, long)]
    output_dir: String,
}

fn load_from_directory(dir: &str) -> Vec<CuboidWithTf> {
    let mut json_files: Vec<String> = vec![];
    let paths = std::fs::read_dir(dir).unwrap();
    for path in paths {
        let path = path.unwrap().path();
        let path_str = path.to_str().unwrap();
        if path_str.ends_with(".json") {
            json_files.push(path_str.to_string());
        }
    }

    if json_files.len() != 1 {
        panic!("There must be exactly 1 JSON files in the input directory");
    }

    let cube_serdes = serde_json::from_str::<Vec<CubeSerde>>(
        &std::fs::read_to_string(&json_files[0]).expect("Failed to read JSON file"),
    )
    .unwrap();

    let cubes: Vec<CuboidWithTf> = cube_serdes
        .iter()
        .map(|t| CuboidWithTf::from_cube_serde(t))
        .collect();

    cubes
}

fn align_axis(cube: &mut CuboidWithTf) {
    // find Z axis
    let mut candidates = vec![];
    let mut temp = cube.clone();
    candidates.push(temp.clone());
    temp.rot_x_90();
    candidates.push(temp.clone());
    temp.rot_x_90();
    candidates.push(temp.clone());
    temp.rot_x_90();
    candidates.push(temp.clone());
    temp.rot_x_90();
    temp.rot_y_90();
    candidates.push(temp.clone());
    temp.rot_y_90();
    temp.rot_y_90();
    candidates.push(temp.clone());

    let mut max_index = 0;
    let mut max_value = f64::MIN;
    for (i, c) in candidates.iter().enumerate() {
        let matrix = c.tf.to_matrix();
        let dot_z = matrix[(2, 2)];
        if dot_z > max_value {
            max_value = dot_z;
            max_index = i;
        }
    }

    temp = candidates[max_index].clone();

    // find X axis
    candidates.clear();
    candidates.push(temp.clone());
    temp.rot_z_90();
    candidates.push(temp.clone());
    temp.rot_z_90();
    candidates.push(temp.clone());
    temp.rot_z_90();
    candidates.push(temp.clone());

    let mut max_index = 0;
    let mut max_value = f64::MIN;
    for (i, c) in candidates.iter().enumerate() {
        let matrix = c.tf.to_matrix();
        let dot_x = matrix[(0, 0)];
        if dot_x > max_value {
            max_value = dot_x;
            max_index = i;
        }
    }

    *cube = candidates[max_index].clone();
}

fn main() {
    let args = Cli::parse();
    let mut cubes = load_from_directory(&args.input_dir);

    for cube in cubes.iter_mut() {
        align_axis(cube);
    }

    let cube_serdes = cubes
        .iter()
        .map(|c| {
            let tf = c.tf.to_homogeneous();
            let size = c.cuboid.half_extents * 2.0;
            CubeSerde {
                tf: tf
                    .row_iter()
                    .map(|row| row.iter().map(|x| *x).collect())
                    .collect(),
                size: size.iter().map(|x| *x).collect(),
            }
        })
        .collect::<Vec<CubeSerde>>();

    let json_data = serde_json::to_string(&cube_serdes).unwrap();
    std::fs::write(format!("{}/output.json", args.output_dir), json_data).unwrap();
}
