use std::panic;

use clap::Parser;
use ncollide3d::bounding_volume::bounding_sphere;
use ncollide3d::bounding_volume::BoundingSphere;
use ncollide3d::na;
use ncollide3d::partitioning::{BVH, BVT};
use ncollide3d::query::visitors::RayInterferencesCollector;
use ncollide3d::query::{Ray, RayCast};
use ncollide3d::shape::{Cuboid, TriMesh};
use serde::{Deserialize, Serialize};
use std::io::Write;
use stl_io::read_stl;

#[derive(Serialize, Deserialize, Debug)]
struct CubeSerde {
    tf: Vec<Vec<f64>>,
    size: Vec<f64>,
}

#[derive(Debug)]
struct CuboidWithTf {
    cuboid: Cuboid<f64>,
    tf: na::Isometry3<f64>,
}

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

    let mut cube_serdes: Vec<CubeSerde> = vec![];
    let mut lidar_serdes: Vec<LidarSerde> = vec![];
    let str1 = std::fs::read_to_string(&json_files[0]).unwrap();
    let str2 = std::fs::read_to_string(&json_files[1]).unwrap();
    if let Ok(cubes) = serde_json::from_str::<Vec<CubeSerde>>(&str1) {
        cube_serdes = cubes;
        lidar_serdes = serde_json::from_str::<Vec<LidarSerde>>(&str2).unwrap();
    } else {
        cube_serdes = serde_json::from_str::<Vec<CubeSerde>>(&str2).unwrap();
        lidar_serdes = serde_json::from_str::<Vec<LidarSerde>>(&str1).unwrap();
    }

    let lidars: Vec<LidarInfo> = lidar_serdes
        .iter()
        .map(|t| {
            let rotmat = na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[
                na::Vector3::new(t.tf[0][0], t.tf[0][1], t.tf[0][2]),
                na::Vector3::new(t.tf[1][0], t.tf[1][1], t.tf[1][2]),
                na::Vector3::new(t.tf[2][0], t.tf[2][1], t.tf[2][2]),
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

    let cubes: Vec<CuboidWithTf> = cube_serdes
        .iter()
        .map(|t| {
            let rotmat = na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[
                na::Vector3::new(t.tf[0][0], t.tf[0][1], t.tf[0][2]),
                na::Vector3::new(t.tf[1][0], t.tf[1][1], t.tf[1][2]),
                na::Vector3::new(t.tf[2][0], t.tf[2][1], t.tf[2][2]),
            ]));
            let translation = na::Translation3::new(t.tf[0][3], t.tf[1][3], t.tf[2][3]);
            let tf = na::Isometry3::from_parts(translation, rotmat.into());
            let size = na::Vector3::new(t.size[0], t.size[1], t.size[2]);
            CuboidWithTf {
                cuboid: Cuboid::new(size / 2.0),
                tf,
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
        let n_verticle = 64;
        let min_vertical = -7f64.to_radians();
        let max_vertical = 52f64.to_radians();
        let n_horizontal = 360;
        let min_horizontal = 0f64.to_radians();
        let max_horizontal = 360f64.to_radians();
        for i in 0..n_verticle {
            for j in 0..n_horizontal {
                let angle_vertical = min_vertical
                    + (max_vertical - min_vertical) * i as f64 / (n_verticle - 1) as f64;
                let angle_horizontal = min_horizontal
                    + (max_horizontal - min_horizontal) * j as f64 / (n_horizontal - 1) as f64;
                let origin = na::Point3::new(0.0, 0.0, 0.0);
                let dir = na::Vector3::new(
                    angle_vertical.cos() * angle_horizontal.cos(),
                    angle_vertical.cos() * angle_horizontal.sin(),
                    angle_vertical.sin(),
                );
                rays.push(Ray::new(origin, dir));
            }
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
) -> Vec<na::Point3<f64>> {
    let mut leaves: Vec<(usize, BoundingSphere<f64>)> = vec![];
    for (index, cube) in cubes.iter().enumerate() {
        leaves.push((
            index,
            bounding_sphere::bounding_sphere(&cube.cuboid, &cube.tf),
        ));
    }

    let bvt = BVT::new_balanced(leaves.clone());
    let mut points: Vec<na::Point3<f64>> = vec![];
    for ray in rays.iter() {
        let mut interferences = Vec::new();
        let mut visitor = RayInterferencesCollector::new(&ray, f64::INFINITY, &mut interferences);
        bvt.visit(&mut visitor);

        let mut smallest_toi = f64::INFINITY;
        for i in interferences.iter() {
            let cube = &cubes[*i];
            let tf = cube.tf;
            let toi = cube.cuboid.toi_with_ray(&tf, &ray, f64::INFINITY, true);
            if let Some(toi) = toi {
                smallest_toi = smallest_toi.min(toi);
            }
        }

        if let Some(scene_mesh) = scene_mesh {
            let toi =
                scene_mesh.toi_with_ray(&na::Isometry3::identity(), &ray, f64::INFINITY, true);
            if let Some(toi) = toi {
                smallest_toi = smallest_toi.min(toi);
            }
        }

        if smallest_toi < f64::INFINITY {
            points.push(ray.origin + ray.dir * smallest_toi);
        }
    }
    points
}

fn write_to_pcd(points: &Vec<na::Point3<f64>>, path: &str) {
    let mut file = std::fs::File::create(path).unwrap();
    file.write_all(b"VERSION .7\n").unwrap();
    file.write_all(format!("FIELDS x y z\n").as_bytes())
        .unwrap();
    file.write_all(format!("SIZE 4 4 4\n").as_bytes()).unwrap();
    file.write_all(format!("TYPE F F F\n").as_bytes()).unwrap();
    file.write_all(format!("COUNT 1 1 1\n").as_bytes()).unwrap();
    file.write_all(b"WIDTH ").unwrap();
    file.write_all(format!("{}\n", points.len()).as_bytes())
        .unwrap();
    file.write_all(b"HEIGHT 1\n").unwrap();
    file.write_all(b"VIEWPOINT 0 0 0 1 0 0 0\n").unwrap();
    file.write_all(b"POINTS ").unwrap();
    file.write_all(format!("{}\n", points.len()).as_bytes())
        .unwrap();
    file.write_all(b"DATA binary\n").unwrap();
    for point in points.iter() {
        file.write_all(&(point.x as f32).to_le_bytes()).unwrap();
        file.write_all(&(point.y as f32).to_le_bytes()).unwrap();
        file.write_all(&(point.z as f32).to_le_bytes()).unwrap();
    }
}

fn main() {
    let args = Cli::parse();
    let (cubes, scene_mesh, lidars) = load_from_directory(&args.input_dir);
    let rays = lidars
        .iter()
        .map(|l| create_rays(l))
        .collect::<Vec<Vec<Ray<f64>>>>();
    let rays: Vec<Ray<f64>> = rays.iter().flatten().cloned().collect();
    let points = ray_casting(&cubes, &scene_mesh, &rays);
    write_to_pcd(&points, &args.output_dir);
}
