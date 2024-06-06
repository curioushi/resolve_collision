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

fn main() {
    let args = Cli::parse();
    let cubes = load_from_directory(&args.input_dir);
    let mut new_cubes: Vec<CuboidWithTf> = vec![];
    for cube in cubes.iter() {
        let mut temp = cube.clone();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();

        temp.rot_x_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();

        temp.rot_x_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();

        temp.rot_x_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        temp.rot_x_90();

        temp.rot_y_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();

        temp.rot_y_90();
        temp.rot_y_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
        temp.rot_z_90();
        new_cubes.push(temp.clone());
    }
}
