use clap::Parser;
use geo::{Area, Coord};
use geo_types::{coord, Rect};
use ncollide3d::bounding_volume::{BoundingVolume, HasBoundingVolume, AABB};
use ncollide3d::na;
use ncollide3d::partitioning::{BVH, BVT};
use ncollide3d::query::visitors::BoundingVolumeInterferencesCollector;
use ncollide3d::query::{self, PointQuery};
use ndarray::Array;
use resolve_collision::common::{Container, CuboidWithTf, Isometry3Serde};
use resolve_collision::gripper::{Gripper, GripperSerde, PickPlan, PickPlanOptions};
use resolve_collision::pcd_io::read_pcd;
use std::collections::HashMap;

const DENSITY_WATER: f64 = 997.0;
const DENSITY_WATER_BOX: f64 = DENSITY_WATER * std::f64::consts::PI / 4.0;
const DENSITY_LIGHT_BOX: f64 = 0.1 * DENSITY_WATER_BOX;

#[derive(Parser, Debug)]
struct Cli {
    #[arg(long)]
    boxes: String,

    #[arg(long)]
    pickable: String,

    #[arg(long)]
    container: String,

    #[arg(long)]
    cloud: Option<String>,

    #[arg(long)]
    gripper: String,

    #[arg(long)]
    options: String,

    #[arg(long)]
    output: String,
}

fn load_from_args(
    args: &Cli,
) -> (
    Vec<CuboidWithTf>,
    Vec<bool>,
    Container,
    Option<Vec<na::Point3<f64>>>,
    Gripper,
    PickPlanOptions,
) {
    let json_file = std::fs::File::open(&args.boxes).expect("Failed to open JSON file");
    let cubes: Vec<CuboidWithTf> =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let json_file = std::fs::File::open(&args.pickable).expect("Failed to open JSON file");
    let pickable_mask: Vec<bool> =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let json_file = std::fs::File::open(&args.container).expect("Failed to open JSON file");
    let container: Container =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    // let json_file = std::fs::File::open(&args.cloud).expect("Failed to open JSON file");
    let pcd = match &args.cloud {
        Some(cloud) => Some(read_pcd(&cloud)),
        None => None,
    };

    let json_file = std::fs::File::open(&args.gripper).expect("Failed to open JSON file");
    let gripper_serde: GripperSerde =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");

    let json_file = std::fs::File::open(&args.options).expect("Failed to open JSON file");
    let mut options: PickPlanOptions = PickPlanOptions::default();
    let new_options: PickPlanOptions =
        serde_json::from_reader(json_file).expect("Failed to read JSON file");
    options.overwrite_from(&new_options);

    let gripper = gripper_serde.to_gripper();

    (cubes, pickable_mask, container, pcd, gripper, options)
}

fn simple_pick<F>(
    cubes: &Vec<CuboidWithTf>,
    pickable_mask: &Vec<bool>,
    gripper: &Gripper,
    collision_cubes: &Vec<CuboidWithTf>,
    bvt: &BVT<usize, AABB<f64>>,
    voxel_map: &Option<VoxelMap>,
    options: &PickPlanOptions,
    initial_guess: F,
) -> Vec<PickPlan>
where
    F: Fn(&CuboidWithTf) -> (Vec<na::Point3<f64>>, na::Isometry3<f64>),
{
    let linear_choice = options.linear_choice.unwrap().clamp(1, 51);
    let min_centrality = options.centrality_filter.unwrap().clamp(0.0, 1.0);

    let mut pick_plans = vec![];
    let suction_areas = gripper.suction_areas();
    let total_suction_area: f64 = suction_areas.iter().sum();
    let suction_rect = gripper.suction_rect();
    for (i, (cube, pickable)) in cubes.iter().zip(pickable_mask.iter()).enumerate() {
        if !pickable {
            continue;
        }
        let hsize = cube.cuboid.half_extents;
        let volume = 8.0 * hsize[0] * hsize[1] * hsize[2];
        let weight = volume * DENSITY_LIGHT_BOX;
        if weight > options.max_payload.unwrap() {
            continue;
        }
        let (face_points, base_tip_tf) = initial_guess(cube);

        let mut pick_plans_of_face = vec![];
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
            let face_area = face_rect.unsigned_area();
            let x1 = face_rect.min().x - suction_rect.max().x;
            let x2 = face_rect.max().x - suction_rect.min().x;
            let step_x = (x2 - x1) / (linear_choice + 1) as f64;
            let y1 = face_rect.min().y - suction_rect.max().y;
            let y2 = face_rect.max().y - suction_rect.min().y;
            let step_y = (y2 - y1) / (linear_choice + 1) as f64;
            for dx in Array::linspace(x1 + step_x, x2 - step_x, linear_choice).iter() {
                for dy in Array::linspace(y1 + step_y, y2 - step_y, linear_choice).iter() {
                    let tf_offset = na::Isometry3::from_parts(
                        na::Translation3::new(*dx, *dy, 0.0),
                        na::UnitQuaternion::identity(),
                    );

                    // turn off suction groups if intersection area is too small
                    let inter_area_centers =
                        gripper.intersection_area_centers(&face_rect, *dx, *dy);
                    let mut inter_areas = inter_area_centers
                        .iter()
                        .map(|(a, _, _)| *a)
                        .collect::<Vec<f64>>();
                    let inter_percents: Vec<f64> = inter_areas
                        .iter()
                        .zip(suction_areas.iter())
                        .map(|(a, b)| a / b)
                        .collect();
                    let suction_group_mask: Vec<bool> =
                        inter_percents.iter().map(|x| x > &0.1).collect(); // think about this threshold
                    if !suction_group_mask.iter().any(|x| *x) {
                        continue;
                    }

                    // compute score
                    inter_areas
                        .iter_mut()
                        .zip(suction_group_mask.iter())
                        .for_each(|(a, b)| {
                            if !*b {
                                *a = 0.0;
                            }
                        });
                    let total_inter_area: f64 = inter_areas.iter().sum();
                    let iou = total_inter_area / total_suction_area.min(face_area);
                    let force_center = inter_areas
                        .iter()
                        .zip(inter_area_centers.iter())
                        .map(|(a, (_, x, y))| *a * na::Vector2::new(*x, *y))
                        .sum::<na::Vector2<f64>>()
                        / total_inter_area;
                    let centrality = 1.0
                        - (force_center.norm() / (face_rect.width().max(face_rect.height()) / 2.0))
                            .min(1.0);
                    if centrality < min_centrality {
                        continue;
                    }
                    let score = (iou + centrality) * 0.5;

                    // TODO: consider error suction
                    // TODO: check weight if other_box_indices is not empty
                    // TODO: merge boxes
                    pick_plans_of_face.push(PickPlan {
                        score,
                        tf_world_tip: Isometry3Serde::new(tf_world_tip * tf_offset),
                        tf_world_flange: Isometry3Serde::new(
                            tf_world_tip * tf_offset * gripper.tf_flange_tip.inverse(),
                        ),
                        suction_group_mask,
                        main_box_index: i,
                        other_box_indices: vec![],
                    });
                }
            }
        }
        pick_plans_of_face.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        pick_plans_of_face = filter_by_collision(
            pick_plans_of_face,
            collision_cubes,
            bvt,
            voxel_map,
            gripper,
            options,
        );
        pick_plans.extend(pick_plans_of_face);
    }
    pick_plans
}

struct VoxelMap {
    map: HashMap<(i32, i32, i32), Vec<usize>>,
    voxel_size: f64,
    points: Vec<na::Point3<f64>>,
}

impl VoxelMap {
    fn new(points: Vec<na::Point3<f64>>, voxel_size: f64) -> Self {
        let mut map = HashMap::new();
        for (i, p) in points.iter().enumerate() {
            let key = (
                (p.x / voxel_size).floor() as i32,
                (p.y / voxel_size).floor() as i32,
                (p.z / voxel_size).floor() as i32,
            );
            map.entry(key).or_insert(vec![]).push(i);
        }
        Self {
            map,
            voxel_size,
            points,
        }
    }

    fn aabb(&self, key: &(i32, i32, i32)) -> AABB<f64> {
        let mins = na::Point3::new(
            key.0 as f64 * self.voxel_size,
            key.1 as f64 * self.voxel_size,
            key.2 as f64 * self.voxel_size,
        );
        let maxs = na::Point3::new(
            (key.0 + 1) as f64 * self.voxel_size,
            (key.1 + 1) as f64 * self.voxel_size,
            (key.2 + 1) as f64 * self.voxel_size,
        );
        AABB::new(mins, maxs)
    }
}

fn filter_by_collision(
    pick_plans: Vec<PickPlan>,
    cubes: &Vec<CuboidWithTf>,
    bvt: &BVT<usize, AABB<f64>>,
    voxel_map: &Option<VoxelMap>,
    gripper: &Gripper,
    options: &PickPlanOptions,
) -> Vec<PickPlan> {
    let max_plans_per_face = options.max_plans_per_face.unwrap();
    if options.dsafe.unwrap() < 0.0 {
        return pick_plans.into_iter().take(max_plans_per_face).collect();
    }
    let dsafe = 1e-4; // we have loosen the collision shape before, so we can use a small value here

    let mut collision_free_plans: Vec<PickPlan> = vec![];
    for pick_plan in pick_plans.into_iter() {
        if collision_free_plans.len() >= max_plans_per_face {
            break;
        }
        let mut collision = false;
        let bv: AABB<f64> = gripper
            .collision_shape
            .bounding_volume(&pick_plan.tf_world_flange);

        // check gripper-box collision
        let mut inters: Vec<usize> = vec![];
        let mut visitor = BoundingVolumeInterferencesCollector::new(&bv, &mut inters);
        bvt.visit(&mut visitor);
        for j in inters.iter() {
            if pick_plan.main_box_index == *j {
                continue;
            }
            let distance = query::distance_composite_shape_shape(
                pick_plan.tf_world_flange.as_ref(),
                &gripper.collision_shape,
                cubes[*j].tf.as_ref(),
                cubes[*j].cuboid.as_ref(),
            );
            if distance <= dsafe {
                collision = true;
                break;
            }
        }
        if collision {
            continue;
        }

        // check gripper-cloud collision
        if let Some(voxel_map) = voxel_map {
            for (k, v) in voxel_map.map.iter() {
                let voxel_bv = voxel_map.aabb(k);
                if voxel_bv.intersects(&bv) {
                    for pi in v.iter() {
                        // point-aabb distance
                        let pt = voxel_map.points[*pi];
                        let mins_pt = bv.mins - pt;
                        let pt_maxs = pt - bv.maxs;
                        let shift = mins_pt.sup(&pt_maxs).sup(&na::zero());
                        let distance_approx = shift.norm();
                        if distance_approx <= dsafe {
                            let distance = gripper.collision_shape.distance_to_point(
                                pick_plan.tf_world_flange.as_ref(),
                                &pt,
                                true,
                            );
                            if distance <= dsafe {
                                collision = true;
                                break;
                            }
                        }
                    }
                    if collision {
                        break;
                    }
                }
            }
        }

        if !collision {
            collision_free_plans.push(pick_plan);
        }
    }
    collision_free_plans
}

fn main() {
    println!("=========================== PickPlan ===========================");
    let time1 = std::time::Instant::now();
    let args = Cli::parse();
    let (cubes, pickable_mask, container, pcd, mut gripper, options) = load_from_args(&args);
    // TODO: filter out unreachable boxes first
    println!("Options: {:?}", options);

    let time2 = std::time::Instant::now();
    // prepare collision check data structure

    // loosen the gripper collision shape at X/Y axis
    let dsafe = options.dsafe.unwrap();
    gripper.loosen_collision_shape(dsafe.max(0.0), dsafe.max(0.0), 0.0);

    let voxel_map = match pcd {
        Some(pcd) => {
            println!("Use cloud data");
            Some(VoxelMap::new(pcd, options.voxel_size.unwrap()))
        }
        None => {
            println!("No cloud data");
            None
        }
    };

    let mut collision_cubes = cubes.clone();
    for part in container.collision_parts().into_iter() {
        collision_cubes.push(part);
    }
    let mut leaves: Vec<(usize, AABB<f64>)> = vec![];
    for (i, c) in collision_cubes.iter().enumerate() {
        let aabb: AABB<f64> = c.cuboid.bounding_volume(&c.tf);
        leaves.push((i, aabb));
    }
    let bvt = BVT::new_balanced(leaves.clone());
    let time3 = std::time::Instant::now();

    // top pick
    let top_pick_plans = simple_pick(
        &cubes,
        &pickable_mask,
        &gripper,
        &collision_cubes,
        &bvt,
        &voxel_map,
        &options,
        |c| {
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
        },
    );
    let time4 = std::time::Instant::now();

    // side pick
    let side_pick_plans = simple_pick(
        &cubes,
        &pickable_mask,
        &gripper,
        &collision_cubes,
        &bvt,
        &voxel_map,
        &options,
        |c| {
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
        },
    );
    let time5 = std::time::Instant::now();
    let pick_plans: Vec<PickPlan> = top_pick_plans
        .into_iter()
        .chain(side_pick_plans.into_iter())
        .collect();
    let time6 = std::time::Instant::now();

    let output_file = std::fs::File::create(&args.output).expect("Failed to create JSON file");
    serde_json::to_writer(output_file, &pick_plans).expect("Failed to write JSON file");
    let time7 = std::time::Instant::now();
    println!("CLI + Load Json time: {:?}", time2 - time1);
    println!("Prepare Collision time: {:?}", time3 - time2);
    println!("Top Pick time: {:?}", time4 - time3);
    println!("Side Pick time: {:?}", time5 - time4);
    println!("Concat time: {:?}", time6 - time5);
    println!("Deserialize + Write time: {:?}", time7 - time6);
}
