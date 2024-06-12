use clap::Parser;
use geo::Coord;
use geo_types::{coord, Rect};
use ncollide3d::na;
use ndarray::Array;
use resolve_collision::common::CuboidWithTf;
use resolve_collision::gripper::{Gripper, GripperSerde, PickPlan, PickPlanSerde};

const DENSITY_WATER: f64 = 997.0;
const DENSITY_WATER_BOX: f64 = DENSITY_WATER * std::f64::consts::PI / 4.0;
const DENSITY_LIGHT_BOX: f64 = 0.3 * DENSITY_WATER_BOX;

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
    let cubes: Vec<CuboidWithTf> =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let json_file = std::fs::File::open(&args.pickable).expect("Failed to open JSON file");
    let pickable_mask: Vec<bool> =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let json_file = std::fs::File::open(&args.gripper).expect("Failed to open JSON file");
    let gripper_serde: GripperSerde =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let gripper = gripper_serde.to_gripper();

    (cubes, pickable_mask, gripper)
}

fn simple_pick<F>(
    cubes: &Vec<CuboidWithTf>,
    pickable_mask: &Vec<bool>,
    gripper: &Gripper,
    initial_guess: F,
) -> Vec<PickPlan>
where
    F: Fn(&CuboidWithTf) -> (Vec<na::Point3<f64>>, na::Isometry3<f64>),
{
    let mut pick_plans = vec![];
    let suction_areas = gripper.suction_areas();
    let suction_rect = gripper.suction_rect();
    for (i, (cube, pickable)) in cubes.iter().zip(pickable_mask.iter()).enumerate() {
        if !pickable {
            continue;
        }
        let hsize = cube.cuboid.half_extents;
        let volume = 8.0 * hsize[0] * hsize[1] * hsize[2];
        let weight = volume * DENSITY_LIGHT_BOX;
        println!("Weight: {}", weight);
        if weight > gripper.max_payload {
            continue;
        }
        let (face_points, base_tip_tf) = initial_guess(cube);
        // assert gripper towards Z axis
        for rot_z in vec![
            0.0,
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI,
            std::f64::consts::PI + std::f64::consts::FRAC_PI_2,
        ] {
            let rot_z = na::Isometry3::from_parts(
                na::Translation3::new(0.0, 0.0, 0.0),
                na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), rot_z),
            );
            let tf_world_tip = base_tip_tf * rot_z;
            let face_points_proj = face_points
                .iter()
                .map(|p| {
                    let p = tf_world_tip.inverse_transform_point(p);
                    coord! {x: p.x, y: p.y}
                })
                .collect::<Vec<Coord>>();
            let mut min_x = f64::MAX;
            let mut min_y = f64::MAX;
            let mut max_x = f64::MIN;
            let mut max_y = f64::MIN;
            for p in face_points_proj.iter() {
                min_x = min_x.min(p.x);
                min_y = min_y.min(p.y);
                max_x = max_x.max(p.x);
                max_y = max_y.max(p.y);
            }
            let face_rect = Rect::new(coord! {x: min_x, y: min_y}, coord! {x: max_x, y: max_y});
            let dx1 = face_rect.min().x - suction_rect.min().x;
            let dx2 = face_rect.max().x - suction_rect.max().x;
            let (dx1, dx2) = match dx1 < dx2 {
                true => (dx1, dx2),
                false => (dx2, dx1),
            };
            let dx_list = match suction_rect.width() > face_rect.width() {
                true => Array::linspace(dx1, dx2, 3),
                false => Array::from_vec(vec![(dx1 + dx2) / 2.0]),
            };
            let dy1 = face_rect.min().y - suction_rect.min().y;
            let dy2 = face_rect.max().y - suction_rect.max().y;
            let (dy1, dy2) = match dy1 < dy2 {
                true => (dy1, dy2),
                false => (dy2, dy1),
            };
            let dy_list = match suction_rect.height() > face_rect.height() {
                true => Array::linspace(dy1, dy2, 3),
                false => Array::from_vec(vec![(dy1 + dy2) / 2.0]),
            };
            for dx in dx_list.iter() {
                for dy in dy_list.iter() {
                    let tf_offset = na::Isometry3::from_parts(
                        na::Translation3::new(*dx, *dy, 0.0),
                        na::UnitQuaternion::identity(),
                    );
                    let inter_areas = gripper.intersection_areas(&face_rect.to_polygon(), *dx, *dy);
                    let inter_percents: Vec<f64> = inter_areas
                        .iter()
                        .zip(suction_areas.iter())
                        .map(|(a, b)| a / b)
                        .collect();
                    let suction_group_mask: Vec<bool> =
                        inter_percents.iter().map(|x| x > &0.5).collect();

                    // TODO: check weight if other_box_indices is not empty
                    pick_plans.push(PickPlan {
                        score: 1.0,
                        tf_world_tip: tf_world_tip * tf_offset,
                        tf_world_flange: tf_world_tip * tf_offset * gripper.tf_flange_tip.inverse(),
                        suction_group_mask,
                        main_box_index: i,
                        other_box_indices: vec![],
                    });
                }
            }
        }
    }
    pick_plans
}

fn main() {
    let time1 = std::time::Instant::now();
    let args = Cli::parse();
    let (cubes, pickable_mask, gripper) = load_from_args(&args);

    let time2 = std::time::Instant::now();
    let top_pick_plans = simple_pick(&cubes, &pickable_mask, &gripper, |c| {
        let hsize = c.cuboid.half_extents;
        let face_points = vec![
            c.tf.transform_point(&na::Point3::new(hsize[0], hsize[1], hsize[2])),
            c.tf.transform_point(&na::Point3::new(-hsize[0], hsize[1], hsize[2])),
            c.tf.transform_point(&na::Point3::new(-hsize[0], -hsize[1], hsize[2])),
            c.tf.transform_point(&na::Point3::new(hsize[0], -hsize[1], hsize[2])),
        ];
        let position = c.tf.transform_point(&na::Point3::new(0.0, 0.0, hsize[2]));
        let base_tip_tf = na::Isometry3::from_parts(
            na::Translation3::new(position.x, position.y, position.z),
            c.tf.rotation
                * na::Rotation3::from_axis_angle(&na::Vector3::x_axis(), std::f64::consts::PI),
        );
        (face_points, base_tip_tf)
    });
    let side_pick_plans = simple_pick(&cubes, &pickable_mask, &gripper, |c| {
        let hsize = c.cuboid.half_extents;
        let face_points = vec![
            c.tf.transform_point(&na::Point3::new(-hsize[0], hsize[1], hsize[2])),
            c.tf.transform_point(&na::Point3::new(-hsize[0], -hsize[1], hsize[2])),
            c.tf.transform_point(&na::Point3::new(-hsize[0], -hsize[1], -hsize[2])),
            c.tf.transform_point(&na::Point3::new(-hsize[0], hsize[1], -hsize[2])),
        ];
        let position = c.tf.transform_point(&na::Point3::new(-hsize[0], 0.0, 0.0));
        let base_tip_tf = na::Isometry3::from_parts(
            na::Translation3::new(position.x, position.y, position.z),
            c.tf.rotation
                * na::Rotation3::from_axis_angle(
                    &na::Vector3::y_axis(),
                    std::f64::consts::FRAC_PI_2,
                ),
        );
        (face_points, base_tip_tf)
    });
    let pick_plans: Vec<PickPlan> = top_pick_plans
        .into_iter()
        .chain(side_pick_plans.into_iter())
        .collect();
    let time3 = std::time::Instant::now();

    let pick_plan_serdes: Vec<PickPlanSerde> = pick_plans
        .iter()
        .map(|p| PickPlanSerde::from_pick_plan(p))
        .collect();
    let output_file = std::fs::File::create(&args.output).expect("Failed to create JSON file");
    serde_json::to_writer(output_file, &pick_plan_serdes).expect("Failed to write JSON file");
    let time4 = std::time::Instant::now();
    println!("CLI + Load Json time: {:?}", time2 - time1);
    println!("Pick Plan time: {:?}", time3 - time2);
    println!("Deserialize + Write time: {:?}", time4 - time3);
}
