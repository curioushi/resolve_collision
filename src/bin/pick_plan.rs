use clap::Parser;
use geo::Area;
use geo::BooleanOps;
use geo_types::{coord, LineString, MultiPolygon, Polygon, Rect};
use ncollide3d::na;
use resolve_collision::common::{CubeSerde, CuboidWithTf};
use resolve_collision::gripper::{Gripper, GripperSerde, PickPlan, PickPlanSerde};

#[derive(Parser, Debug)]
struct Cli {
    #[arg(short, long)]
    boxes: String,

    #[arg(short, long)]
    pickable: String,

    #[arg(short, long)]
    gripper: String,

    #[arg(short, long)]
    output: String,
}

fn load_from_args(args: &Cli) -> (Vec<CuboidWithTf>, Vec<bool>, Gripper) {
    let json_file = std::fs::File::open(&args.boxes).expect("Failed to open JSON file");
    let cubes_serde: Vec<CubeSerde> =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let mut cubes = vec![];
    for c in cubes_serde.iter() {
        cubes.push(CuboidWithTf::from_cube_serde(c));
    }

    let json_file = std::fs::File::open(&args.pickable).expect("Failed to open JSON file");
    let pickable_mask: Vec<bool> =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let json_file = std::fs::File::open(&args.gripper).expect("Failed to open JSON file");
    let gripper_serde: GripperSerde =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let gripper = gripper_serde.to_gripper();

    (cubes, pickable_mask, gripper)
}

fn top_pick(
    cubes: &Vec<CuboidWithTf>,
    pickable_mask: &Vec<bool>,
    gripper: &Gripper,
) -> Vec<PickPlan> {
    let mut pick_plans = vec![];
    for (i, (cube, pickable)) in cubes.iter().zip(pickable_mask.iter()).enumerate() {
        if !pickable {
            continue;
        }
        let half_size = cube.cuboid.half_extents;
        let base_tip_point = cube
            .tf
            .transform_point(&na::Point3::new(0.0, 0.0, half_size[2]));
        // assert gripper towards Z axis
        let base_tip_rotation = cube.tf.rotation
            * na::Rotation3::from_axis_angle(&na::Vector3::x_axis(), std::f64::consts::PI);
        let base_tip_tf = na::Isometry3::from_parts(
            na::Translation3::new(base_tip_point[0], base_tip_point[1], base_tip_point[2]),
            base_tip_rotation,
        );
        pick_plans.push(PickPlan {
            score: 1.0,
            tf_world_tip: base_tip_tf,
            suction_group_indices: vec![0],
            main_box_index: i,
            other_box_indices: vec![],
        });
    }
    pick_plans
}

fn main() {
    let start_time = std::time::Instant::now();
    let args = Cli::parse();
    let (cubes, pickable_mask, gripper) = load_from_args(&args);
    let pick_plans = top_pick(&cubes, &pickable_mask, &gripper);
    let pick_plan_serdes: Vec<PickPlanSerde> = pick_plans
        .iter()
        .map(|p| PickPlanSerde::from_pick_plan(p))
        .collect();
    let output_file = std::fs::File::create(&args.output).expect("Failed to create JSON file");
    serde_json::to_writer(output_file, &pick_plan_serdes).expect("Failed to write JSON file");
    let end_time = std::time::Instant::now();
    println!("Elapsed time: {:?}", end_time - start_time);
}
