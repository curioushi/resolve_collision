use std::panic;

use clap::Parser;
use ncollide3d::bounding_volume::bounding_sphere;
use ncollide3d::bounding_volume::BoundingSphere;
use ncollide3d::bounding_volume::AABB;
use ncollide3d::na;
use ncollide3d::partitioning::{BVH, BVT};
use ncollide3d::query::visitors::RayInterferencesCollector;
use ncollide3d::query::{Ray, RayCast};
use ncollide3d::shape::TriMesh;
use resolve_collision::common::CuboidWithTf;
use resolve_collision::pcd_io::write_pcd_with_normal;
use serde::{Deserialize, Serialize};
use std::io::Write;
use stl_io::read_stl;

#[derive(Serialize, Deserialize, Debug)]
struct LidarSerde {
    tf: Vec<Vec<f64>>,
    model: String,
    num_frames: Option<usize>,
}

#[derive(Debug)]
struct LidarInfo {
    tf: na::Isometry3<f64>,
    model: String,
    num_frames: Option<usize>,
}

#[derive(Parser, Debug)]
struct Cli {
    #[arg(short, long)]
    input_dir: String,

    #[arg(short, long)]
    output_dir: String,
}

fn load_from_directory(dir: &str) -> (Vec<CuboidWithTf>, Option<TriMesh<f64>>, Vec<LidarInfo>) {
    let mut json_files: Vec<String> = vec![];
    let mut stl_files: Vec<String> = vec![];
    let paths = std::fs::read_dir(dir).unwrap();
    for path in paths {
        let path = path.unwrap().path();
        let path_str = path.to_str().unwrap();
        if path_str.ends_with(".json") {
            json_files.push(path_str.to_string());
        } else if path_str.ends_with(".stl") {
            stl_files.push(path_str.to_string());
        }
    }

    if json_files.len() != 2 {
        panic!("There must be exactly 2 JSON files in the input directory");
    }

    let str1 = std::fs::read_to_string(&json_files[0]).unwrap();
    let str2 = std::fs::read_to_string(&json_files[1]).unwrap();
    let (cubes, lidar_serdes) = match serde_json::from_str::<Vec<CuboidWithTf>>(&str1) {
        Ok(temp) => (
            temp,
            serde_json::from_str::<Vec<LidarSerde>>(&str2).unwrap(),
        ),
        Err(_) => (
            serde_json::from_str::<Vec<CuboidWithTf>>(&str2).unwrap(),
            serde_json::from_str::<Vec<LidarSerde>>(&str1).unwrap(),
        ),
    };

    let lidars: Vec<LidarInfo> = lidar_serdes
        .iter()
        .map(|t| {
            let rotmat = na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[
                na::Vector3::new(t.tf[0][0], t.tf[1][0], t.tf[2][0]),
                na::Vector3::new(t.tf[0][1], t.tf[1][1], t.tf[2][1]),
                na::Vector3::new(t.tf[0][2], t.tf[1][2], t.tf[2][2]),
            ]));
            let translation = na::Translation3::new(t.tf[0][3], t.tf[1][3], t.tf[2][3]);
            let tf = na::Isometry3::from_parts(translation, rotmat.into());
            LidarInfo {
                tf,
                model: t.model.clone(),
                num_frames: t.num_frames,
            }
        })
        .collect();

    if stl_files.len() == 1 {
        let file = std::fs::File::open(&stl_files[0]).unwrap();
        let mut reader = std::io::BufReader::new(file);
        let stl = read_stl(&mut reader).expect("Failed to read STL file");

        let vertices = stl
            .vertices
            .iter()
            .map(|v| na::Point3::new(v[0] as f64, v[1] as f64, v[2] as f64))
            .collect();
        let indices = stl
            .faces
            .iter()
            .map(|f| na::Point3::new(f.vertices[0], f.vertices[1], f.vertices[2]))
            .collect();

        let scene_mesh = TriMesh::new(vertices, indices, None);
        (cubes, Some(scene_mesh), lidars)
    } else {
        (cubes, None, lidars)
    }
}

fn create_rays(lidar_info: &LidarInfo) -> Vec<Ray<f64>> {
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

fn ray_casting(
    cubes: &Vec<CuboidWithTf>,
    scene_mesh: &Option<TriMesh<f64>>,
    rays: &Vec<Ray<f64>>,
) -> (
    Vec<na::Point3<f64>>,
    Vec<na::Vector3<f64>>,
    Vec<Vec<na::Point3<f64>>>,
) {
    let mut leaves: Vec<(usize, BoundingSphere<f64>)> = vec![];
    for (index, cube) in cubes.iter().enumerate() {
        leaves.push((
            index,
            bounding_sphere::bounding_sphere(cube.cuboid.as_ref(), &cube.tf),
        ));
    }

    let bvt = BVT::new_balanced(leaves.clone());
    let mut points: Vec<na::Point3<f64>> = vec![];
    let mut normals: Vec<na::Vector3<f64>> = vec![];
    let mut cube_points: Vec<Vec<na::Point3<f64>>> = vec![vec![]; cubes.len()];
    for ray in rays.iter() {
        let mut interferences = Vec::new();
        let mut visitor = RayInterferencesCollector::new(&ray, f64::INFINITY, &mut interferences);
        bvt.visit(&mut visitor);

        let mut smallest_toi = f64::INFINITY;
        let mut cube_index: Option<usize> = None;
        for i in interferences.iter() {
            let cube = &cubes[*i];
            let tf = cube.tf.as_ref();
            let toi = cube.cuboid.toi_with_ray(&tf, &ray, f64::INFINITY, true);
            if let Some(toi) = toi {
                if toi < smallest_toi {
                    smallest_toi = toi;
                    cube_index = Some(*i);
                }
            }
        }

        if let Some(scene_mesh) = scene_mesh {
            let toi =
                scene_mesh.toi_with_ray(&na::Isometry3::identity(), &ray, f64::INFINITY, true);
            if let Some(toi) = toi {
                if toi < smallest_toi {
                    smallest_toi = toi;
                    cube_index = None;
                }
            }
        }

        if smallest_toi < f64::INFINITY {
            let point = ray.origin + ray.dir * smallest_toi;
            points.push(point);
            normals.push(ray.dir);
            if let Some(cube_index) = cube_index {
                cube_points[cube_index].push(point);
            }
        }
    }
    (points, normals, cube_points)
}

fn compute_visibility(cube: &CuboidWithTf, cube_points: &Vec<na::Point3<f64>>) -> f64 {
    if cube_points.len() == 0 {
        return 0.0;
    }
    let cube_points = cube_points
        .iter()
        .map(|p| cube.tf.inverse_transform_point(p));
    let mut front_points: Vec<na::Point3<f64>> = vec![];
    let mut back_points: Vec<na::Point3<f64>> = vec![];
    let mut left_points: Vec<na::Point3<f64>> = vec![];
    let mut right_points: Vec<na::Point3<f64>> = vec![];
    let mut top_points: Vec<na::Point3<f64>> = vec![];
    let mut bottom_points: Vec<na::Point3<f64>> = vec![];
    let (half_x, half_y, half_z) = (
        cube.cuboid.half_extents[0],
        cube.cuboid.half_extents[1],
        cube.cuboid.half_extents[2],
    );
    for point in cube_points {
        let (x, y, z) = (point.x, point.y, point.z);
        if (x - half_x).abs() < 1e-4 {
            front_points.push(point);
        } else if (x + half_x).abs() < 1e-4 {
            back_points.push(point);
        } else if (y - half_y).abs() < 1e-4 {
            left_points.push(point);
        } else if (y + half_y).abs() < 1e-4 {
            right_points.push(point);
        } else if (z - half_z).abs() < 1e-4 {
            top_points.push(point);
        } else if (z + half_z).abs() < 1e-4 {
            bottom_points.push(point);
        }
    }
    // compute area of each face
    let mut front_iou = 0.0;
    let mut back_iou = 0.0;
    let mut left_iou = 0.0;
    let mut right_iou = 0.0;
    let mut top_iou = 0.0;
    let mut bottom_iou = 0.0;
    if front_points.len() > 1 {
        let front_aabb = AABB::from_points(&front_points);
        front_iou =
            (front_aabb.half_extents()[1] * front_aabb.half_extents()[2]) / (half_y * half_z);
    }
    if back_points.len() > 1 {
        let back_aabb = AABB::from_points(&back_points);
        back_iou = (back_aabb.half_extents()[1] * back_aabb.half_extents()[2]) / (half_y * half_z);
    }
    if left_points.len() > 1 {
        let left_aabb = AABB::from_points(&left_points);
        left_iou = (left_aabb.half_extents()[0] * left_aabb.half_extents()[2]) / (half_x * half_z);
    }
    if right_points.len() > 1 {
        let right_aabb = AABB::from_points(&right_points);
        right_iou =
            (right_aabb.half_extents()[0] * right_aabb.half_extents()[2]) / (half_x * half_z);
    }
    if top_points.len() > 1 {
        let top_aabb = AABB::from_points(&top_points);
        top_iou = (top_aabb.half_extents()[0] * top_aabb.half_extents()[1]) / (half_x * half_y);
    }
    if bottom_points.len() > 1 {
        let bottom_aabb = AABB::from_points(&bottom_points);
        bottom_iou =
            (bottom_aabb.half_extents()[0] * bottom_aabb.half_extents()[1]) / (half_x * half_y);
    }
    front_iou
        .max(back_iou)
        .max(left_iou)
        .max(right_iou)
        .max(top_iou)
        .max(bottom_iou)
}

fn main() {
    println!("=========================== Simulate Lidar ===========================");
    let time1 = std::time::Instant::now();
    let args = Cli::parse();
    let (cubes, scene_mesh, lidars) = load_from_directory(&args.input_dir);
    let time2 = std::time::Instant::now();
    let rays = lidars
        .iter()
        .map(|l| create_rays(l))
        .collect::<Vec<Vec<Ray<f64>>>>();
    let rays: Vec<Ray<f64>> = rays.iter().flatten().cloned().collect();
    let (points, normals, cube_points) = ray_casting(&cubes, &scene_mesh, &rays);
    let time3 = std::time::Instant::now();
    write_pcd_with_normal(&points, &normals, &args.output_dir);

    // write visibility to file
    let visibility: Vec<f64> = cubes
        .iter()
        .zip(cube_points.iter())
        .map(|(cube, cube_points)| compute_visibility(cube, cube_points))
        .collect();
    let mut file = std::fs::File::create(format!("{}/visibility.json", args.output_dir)).unwrap();
    file.write_all(serde_json::to_string(&visibility).unwrap().as_bytes())
        .unwrap();
    let time4 = std::time::Instant::now();
    println!("Deserialize time: {:?}", time2 - time1);
    println!("Ray casting time: {:?}", time3 - time2);
    println!("Serialize time: {:?}", time4 - time3);
}
