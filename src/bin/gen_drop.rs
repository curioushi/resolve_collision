use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::path::{Path, PathBuf};

use clap::Parser;
use ncollide3d::bounding_volume::{HasBoundingVolume, AABB};
use ncollide3d::na;
use ncollide3d::query;
use ncollide3d::query::{Ray, RayCast};
use ncollide3d::shape::{Cuboid, TriMesh};
use ncollide3d::simba::scalar::SubsetOf;
use resolve_collision::common::{Container, CuboidSerde, CuboidWithTf, Isometry3Serde};
use resolve_collision::lidar::{create_rays, LidarInfo};
use resolve_collision::pcd_io::write_pcd;
use serde::{Deserialize, Serialize};
use stl_io::read_stl;

#[derive(Deserialize, Serialize, Debug)]
enum YawMode {
    Random,
    Zero,
}

#[derive(Deserialize, Serialize, Debug)]
enum XYMode {
    Random,
    TouchEdge,
}

#[derive(Deserialize, Serialize, Debug)]
struct Options {
    yaw_mode: YawMode,
    xy_mode: XYMode,
    length_range: [f64; 2],
    width_range: [f64; 2],
    height_range: [f64; 2],
    robot_size: [f64; 3],
    samples_per_scene: usize,
    lidar_infos: Vec<LidarInfo>,
}

#[derive(Parser, Debug)]
struct Cli {
    #[arg(short, long)]
    input: String,

    #[arg(long)]
    output: String,

    #[arg(long)]
    options: String,
}

#[derive(Serialize, Deserialize, Debug)]
struct BoxSerde {
    pose: [[f64; 4]; 4],
    dim: [f64; 3],
}

#[derive(Serialize, Deserialize, Debug)]
struct ContainerSerde {
    tf_base_container: [[f64; 4]; 4],
    container_size: [f64; 3],
    beam_height: f64,
}

fn load_from_input_subdir(dir: PathBuf) -> (Vec<CuboidWithTf>, Container, TriMesh<f64>) {
    let boxes_path = dir.join("boxes.json");
    let container_path = dir.join("container.json");
    let scene_path = dir.join("scene.stl");

    // read boxes.json
    let boxes: Vec<BoxSerde> = serde_json::from_reader(std::fs::File::open(&boxes_path).unwrap())
        .expect("Failed to read boxes.json");
    let boxes: Vec<CuboidWithTf> = boxes
        .iter()
        .map(|b| CuboidWithTf {
            cuboid: CuboidSerde::new(na::Vector3::new(
                b.dim[0] / 2.0,
                b.dim[1] / 2.0,
                b.dim[2] / 2.0,
            )),
            tf: Isometry3Serde::new(na::Isometry3::from_superset_unchecked(&na::Matrix4::new(
                b.pose[0][0],
                b.pose[0][1],
                b.pose[0][2],
                b.pose[0][3],
                b.pose[1][0],
                b.pose[1][1],
                b.pose[1][2],
                b.pose[1][3],
                b.pose[2][0],
                b.pose[2][1],
                b.pose[2][2],
                b.pose[2][3],
                b.pose[3][0],
                b.pose[3][1],
                b.pose[3][2],
                b.pose[3][3],
            ))),
        })
        .collect();

    // read container.json
    let container: ContainerSerde =
        serde_json::from_reader(std::fs::File::open(&container_path).unwrap())
            .expect("Failed to read container.json");
    let container = Container {
        cuboid: CuboidSerde::new(na::Vector3::new(
            container.container_size[0] / 2.0,
            container.container_size[1] / 2.0,
            container.container_size[2] / 2.0,
        )),
        tf: Isometry3Serde::new(na::Isometry3::from_superset_unchecked(&na::Matrix4::new(
            container.tf_base_container[0][0],
            container.tf_base_container[0][1],
            container.tf_base_container[0][2],
            container.tf_base_container[0][3],
            container.tf_base_container[1][0],
            container.tf_base_container[1][1],
            container.tf_base_container[1][2],
            container.tf_base_container[1][3],
            container.tf_base_container[2][0],
            container.tf_base_container[2][1],
            container.tf_base_container[2][2],
            container.tf_base_container[2][3],
            container.tf_base_container[3][0],
            container.tf_base_container[3][1],
            container.tf_base_container[3][2],
            container.tf_base_container[3][3],
        ))),
    };

    // read scene.stl
    let file = std::fs::File::open(&scene_path).unwrap();
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
    (boxes, container, scene_mesh)
}

fn compute_valid_region(
    boxes: &Vec<CuboidWithTf>,
    container: &Container,
) -> (na::Point2<f64>, na::Point2<f64>) {
    let hsize = container.cuboid.half_extents;
    let mut max_x = 0.0f64;
    for b in boxes.iter() {
        let tf = container.tf.inv_mul(&b.tf);
        let aabb: AABB<f64> = b.cuboid.bounding_volume(&tf);
        let x1 = aabb.mins.x;
        let x2 = aabb.maxs.x;
        max_x = max_x.min(x1);
        max_x = max_x.min(x2);
    }
    (
        na::Point2::new(-2.0 * hsize.x, -hsize.y),
        na::Point2::new(max_x, hsize.y),
    )
}

fn sample_dropped_box(
    mins: &na::Point2<f64>,
    maxs: &na::Point2<f64>,
    tf_container_base: &na::Isometry3<f64>,
    options: &Options,
) -> Option<(CuboidWithTf, CuboidWithTf)> {
    let robot = CuboidWithTf {
        cuboid: CuboidSerde::new(na::Vector3::new(
            options.robot_size[0] / 2.0 + 18.0,
            options.robot_size[1] / 2.0,
            options.robot_size[2] / 2.0,
        )),
        tf: Isometry3Serde::new(
            tf_container_base.clone()
                * na::Isometry3::from_parts(
                    na::Translation3::new(-9.0, 0.0, 0.0),
                    na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
                ),
        ),
    };
    let base_x = tf_container_base.translation.x;
    let mins = na::Point2::new(mins.x.max(base_x - 3.0), mins.y);
    let maxs = na::Point2::new(maxs.x.min(base_x + 3.0), maxs.y);
    let mut rng = rand::thread_rng();
    let size_x = rng.gen_range(options.length_range[0]..options.length_range[1]);
    let size_y = rng.gen_range(options.width_range[0]..options.width_range[1]);
    let size_z = rng.gen_range(options.height_range[0]..options.height_range[1]);
    let dim = na::Vector3::new(size_x / 2.0, size_y / 2.0, size_z / 2.0);
    for _ in 0..100 {
        let yaw = match options.yaw_mode {
            YawMode::Random => rng.gen_range(0.0..std::f64::consts::PI),
            YawMode::Zero => 0.0,
        };
        let cuboid = Cuboid::new(dim);
        let aabb: AABB<f64> = cuboid.bounding_volume(&na::Isometry3::from_parts(
            na::Translation3::new(0.0, 0.0, 0.0),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
        ));
        let aabb_size = aabb.maxs - aabb.mins;
        if aabb_size.x > maxs.x - mins.x || aabb_size.y > maxs.y - mins.y {
            continue;
        }
        let mins2 = mins + na::Vector2::new(aabb_size.x / 2.0, aabb_size.y / 2.0);
        let maxs2 = maxs - na::Vector2::new(aabb_size.x / 2.0, aabb_size.y / 2.0);
        let (x, y) = match options.xy_mode {
            XYMode::Random => (
                rng.gen_range(mins2.x..maxs2.x),
                rng.gen_range(mins2.y..maxs2.y),
            ),
            XYMode::TouchEdge => {
                if rng.gen_ratio(1, 2) {
                    (maxs2.x, rng.gen_range(mins2.y..maxs2.y)) // front edge
                } else if rng.gen_ratio(1, 2) {
                    (rng.gen_range(mins2.x..maxs2.x), mins2.y) // left edge
                } else {
                    (rng.gen_range(mins2.x..maxs2.x), maxs2.y) // right edge
                }
            }
        };
        // check box-base collision
        let boxx = CuboidWithTf {
            cuboid: CuboidSerde::new(dim),
            tf: Isometry3Serde::new(na::Isometry3::from_parts(
                na::Translation3::new(x, y, size_z / 2.0),
                na::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
            )),
        };
        if query::proximity(
            &boxx.tf,
            boxx.cuboid.as_ref(),
            &robot.tf,
            robot.cuboid.as_ref(),
            0.0,
        ) == query::Proximity::Disjoint
        {
            let ratio = rng.gen_range(0.1..0.9);
            let mins2 = mins + ratio * na::Vector2::new(aabb_size.x, aabb_size.y);
            let maxs2 = maxs - ratio * na::Vector2::new(aabb_size.x, aabb_size.y);
            let size2 = maxs2 - mins2;
            let center2 = mins2 + size2 / 2.0;
            let region = CuboidWithTf {
                cuboid: CuboidSerde::new(na::Vector3::new(size2.x / 2.0, size2.y / 2.0, 0.35)),
                tf: Isometry3Serde::new(na::Isometry3::from_parts(
                    na::Translation3::new(center2.x, center2.y, 0.35 + 0.1),
                    na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
                )),
            };
            return Some((boxx, region));
        }
    }
    None
}

enum Label {
    Box,
    Robot,
    Scene,
    Unknown,
}

fn ray_casting(
    cube: &CuboidWithTf,
    robot: &CuboidWithTf,
    scene_mesh: &TriMesh<f64>,
    rays: &Vec<Ray<f64>>,
) -> (Vec<na::Point3<f64>>, Vec<na::Vector3<f64>>, Vec<Label>) {
    let mut points: Vec<na::Point3<f64>> = vec![];
    let mut normals: Vec<na::Vector3<f64>> = vec![];
    let mut labels: Vec<Label> = vec![];
    for ray in rays.iter() {
        let mut smallest_toi = f64::INFINITY;
        let mut label = Label::Unknown;
        let toi = cube
            .cuboid
            .toi_with_ray(&cube.tf, &ray, f64::INFINITY, true);
        if let Some(toi) = toi {
            if toi < smallest_toi {
                smallest_toi = toi;
                label = Label::Box;
            }
        }

        let toi = robot
            .cuboid
            .toi_with_ray(&robot.tf, &ray, f64::INFINITY, true);
        if let Some(toi) = toi {
            if toi < smallest_toi {
                smallest_toi = toi;
                label = Label::Robot;
            }
        }

        let toi = scene_mesh.toi_with_ray(&na::Isometry3::identity(), &ray, f64::INFINITY, true);
        if let Some(toi) = toi {
            if toi < smallest_toi {
                smallest_toi = toi;
                label = Label::Scene;
            }
        }

        if smallest_toi < f64::INFINITY {
            match label {
                Label::Box | Label::Scene => {
                    let point = ray.origin + ray.dir * smallest_toi;
                    points.push(point);
                    normals.push(ray.dir);
                    labels.push(label);
                }
                _ => {}
            }
        }
    }
    (points, normals, labels)
}

fn main() {
    let args = Cli::parse();
    let option_file = std::fs::File::open(&args.options).unwrap();
    let options: Options = serde_json::from_reader(option_file).unwrap();
    println!("{:?}", options);

    let rays = options
        .lidar_infos
        .iter()
        .map(|l| create_rays(l))
        .flatten()
        .collect::<Vec<Ray<f64>>>();

    // read all subfolders
    let input_root = args.input;
    let mut subfolders: Vec<std::path::PathBuf> = std::fs::read_dir(&input_root)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| {
            let boxes_path = path.join("boxes.json");
            let container_path = path.join("container.json");
            let scene_path = path.join("scene.stl");
            Path::new(&boxes_path).exists()
                && Path::new(&container_path).exists()
                && Path::new(&scene_path).exists()
        })
        .collect();
    subfolders.sort();

    let robot = CuboidWithTf {
        cuboid: CuboidSerde::new(na::Vector3::new(
            options.robot_size[0] / 2.0,
            options.robot_size[1] / 2.0,
            options.robot_size[2] / 2.0,
        )),
        tf: Isometry3Serde::new(na::Isometry3::from_parts(
            na::Translation3::new(0.0, 0.0, options.robot_size[2] / 2.0),
            na::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        )),
    };

    let mut counter = 0;
    let mut rng = rand::thread_rng();
    for (i, subfolder) in subfolders.iter().enumerate() {
        println!("Processing {}/{}: {:?}", i + 1, subfolders.len(), subfolder);
        let (boxes, container, scene_mesh) = load_from_input_subdir(subfolder.clone());
        // in container frame
        let tf_container_base = container.tf.inverse();
        let (mins, maxs) = compute_valid_region(&boxes, &container);
        let mut j = 0;
        let mut num_try = 0;
        while j < options.samples_per_scene && num_try < 10 {
            num_try += 1;
            let sample = sample_dropped_box(&mins, &maxs, &tf_container_base, &options);
            if let Some((dropped_box, region)) = sample {
                // in base frame
                let dropped_box = CuboidWithTf {
                    cuboid: dropped_box.cuboid,
                    tf: Isometry3Serde::new(tf_container_base.inv_mul(&dropped_box.tf)),
                };
                let region = CuboidWithTf {
                    cuboid: region.cuboid,
                    tf: Isometry3Serde::new(tf_container_base.inv_mul(&region.tf)),
                };
                let (points, normals, _labels) =
                    ray_casting(&dropped_box, &robot, &scene_mesh, &rays);
                let normal = Normal::new(0.0, 0.007).unwrap();
                let points = points
                    .iter()
                    .zip(normals.iter())
                    .map(|(p, n)| p + normal.sample(&mut rng) * n)
                    .collect::<Vec<_>>();
                let output_dir = format!("{}/{:04}", args.output, counter);
                std::fs::create_dir_all(&output_dir).unwrap();
                write_pcd(&points, format!("{}/cloud.pcd", output_dir).as_str());
                let _ = serde_json::to_writer(
                    std::fs::File::create(format!("{}/dropped_box.json", output_dir)).unwrap(),
                    &dropped_box,
                );
                let _ = serde_json::to_writer(
                    std::fs::File::create(format!("{}/region.json", output_dir)).unwrap(),
                    &region,
                );
                j += 1;
                counter += 1;
            }
        }
    }
}
