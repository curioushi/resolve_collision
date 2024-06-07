use std::vec;

use clap::Parser;
use resolve_collision::common::{CubeSerde, CuboidWithTf};

#[derive(Parser, Debug)]
struct Cli {
    #[arg(short, long)]
    input: String,

    #[arg(short, long)]
    output: String,
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
    let start_time = std::time::Instant::now();
    let args = Cli::parse();

    // load
    let cube_serdes = serde_json::from_str::<Vec<CubeSerde>>(
        &std::fs::read_to_string(&args.input).expect("Failed to read JSON file"),
    )
    .unwrap();

    let mut cubes: Vec<CuboidWithTf> = cube_serdes
        .iter()
        .map(|t| CuboidWithTf::from_cube_serde(t))
        .collect();

    // align
    for cube in cubes.iter_mut() {
        align_axis(cube);
    }

    // save
    let cube_serdes = cubes
        .iter()
        .map(|c| CubeSerde::from_cube(c))
        .collect::<Vec<CubeSerde>>();

    let json_data = serde_json::to_string(&cube_serdes).unwrap();
    std::fs::write(args.output, json_data).unwrap();
    let duration = start_time.elapsed();
    println!("Time elapsed: {:?}", duration);
}
