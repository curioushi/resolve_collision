use clap::Parser;
use ncollide3d::bounding_volume::{bounding_sphere, BoundingSphere, BoundingVolume};
use ncollide3d::na;
use ncollide3d::partitioning::{BVH, BVT};
use ncollide3d::query::visitors::PointInterferencesCollector;
use ncollide3d::query::PointQuery;
use ncollide3d::shape::{Cuboid, TriMesh};
use resolve_collision::pcd_io::{read_pcd, write_pcd};
use serde::{Deserialize, Serialize};
use std::vec;
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

#[derive(Parser, Debug)]
struct Cli {
    #[arg(short, long)]
    input_dir: String,

    #[arg(short, long, default_value = "0.01")]
    distance: f64,

    #[arg(short, long)]
    output_dir: String,
}

fn load_from_directory(
    dir: &str,
) -> (
    Vec<CuboidWithTf>,
    Option<TriMesh<f64>>,
    Vec<na::Point3<f64>>,
) {
    let mut json_files: Vec<String> = vec![];
    let mut stl_files: Vec<String> = vec![];
    let mut pcd_files: Vec<String> = vec![];
    let paths = std::fs::read_dir(dir).unwrap();
    for path in paths {
        let path = path.unwrap().path();
        let path_str = path.to_str().unwrap();
        if path_str.ends_with(".json") {
            json_files.push(path_str.to_string());
        } else if path_str.ends_with(".stl") {
            stl_files.push(path_str.to_string());
        } else if path_str.ends_with(".pcd") {
            pcd_files.push(path_str.to_string());
        }
    }

    if json_files.len() != 1 {
        panic!("There must be exactly 2 JSON files in the input directory");
    }

    let cube_serdes = serde_json::from_str::<Vec<CubeSerde>>(
        &std::fs::read_to_string(&json_files[0]).expect("Failed to read JSON file"),
    )
    .unwrap();

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

    let mut pc: Vec<na::Point3<f64>> = vec![];
    if pcd_files.len() == 1 {
        pc = read_pcd(&pcd_files[0]);
    }

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
        (cubes, Some(scene_mesh), pc)
    } else {
        (cubes, None, pc)
    }
}

fn remove_points_near(
    cubes: &Vec<CuboidWithTf>,
    scene_mesh: &Option<TriMesh<f64>>,
    pc: &Vec<na::Point3<f64>>,
    margin: f64,
) -> (Vec<na::Point3<f64>>, Vec<na::Point3<f64>>) {
    let mut points_to_keep: Vec<na::Point3<f64>> = vec![];
    let mut points_to_remove: Vec<na::Point3<f64>> = vec![];
    let mut leaves: Vec<(usize, BoundingSphere<f64>)> = vec![];
    for (index, cube) in cubes.iter().enumerate() {
        let mut bs = bounding_sphere(&cube.cuboid, &cube.tf);
        bs.loosen(margin);
        leaves.push((index, bs));
    }
    let bvt = BVT::new_balanced(leaves.clone());
    for point in pc.iter() {
        let mut interferences: Vec<usize> = Vec::new();
        let mut visitor = PointInterferencesCollector::new(point, &mut interferences);
        bvt.visit(&mut visitor);

        let mut removed = false;
        for i in interferences.iter() {
            let cube = &cubes[*i];
            let distance = cube.cuboid.distance_to_point(&cube.tf, &point, true);
            if distance < margin {
                points_to_remove.push(*point);
                removed = true;
                break;
            }
        }
        if removed {
            continue;
        }

        if let Some(scene_mesh) = scene_mesh {
            let distance = scene_mesh.distance_to_point(&na::Isometry3::identity(), &point, true);
            if distance < margin {
                points_to_remove.push(*point);
                continue;
            }
        }

        points_to_keep.push(*point);
    }

    (points_to_keep, points_to_remove)
}

fn main() {
    let args = Cli::parse();
    let (cubes, scene_mesh, pc) = load_from_directory(&args.input_dir);
    let (points_to_keep, points_to_remove) =
        remove_points_near(&cubes, &scene_mesh, &pc, args.distance);
    write_pcd(
        &points_to_keep,
        &format!("{}/points_to_keep.pcd", args.output_dir),
    );
    write_pcd(
        &points_to_remove,
        &format!("{}/points_to_remove.pcd", args.output_dir),
    );
}
