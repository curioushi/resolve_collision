use rand::Rng;
use rerun::components::ClearIsRecursive;
use tonic::{transport::Server, Request, Response, Status};

use ncollide3d::na;
use sim_world::sim_world_server::{SimWorld, SimWorldServer};
use sim_world::{BinPackingMethod, Empty, InitRobotRequest, InitSceneRequest};

mod geometry;
use geometry::{bake_cuboids_by_free_fall, simple_stacking, to_rerun_mesh3d, SceneTree, AABB};
use log::{error, info, warn};
use std::ops::Range;
use std::sync::{Arc, Mutex};

pub mod sim_world {
    tonic::include_proto!("sim_world");
}

pub struct MySimWorld {
    rec: Option<rerun::RecordingStream>,
    scene_tree: Arc<Mutex<SceneTree>>,
}

impl MySimWorld {
    fn new() -> Self {
        let scene_tree = Arc::new(Mutex::new(SceneTree::new()));
        let rec = if let Ok(recording) = rerun::RecordingStreamBuilder::new("sim_world").spawn() {
            Some(recording)
        } else {
            None
        };
        Self { rec, scene_tree }
    }
}

fn fixed_range(range: Range<f32>) -> Range<f32> {
    let (min, max) = (range.start, range.end);
    if min < max {
        Range {
            start: min,
            end: max,
        }
    } else {
        Range {
            start: max,
            end: min + 1e-6f32,
        }
    }
}

#[tonic::async_trait]
impl SimWorld for MySimWorld {
    async fn init_scene(
        &self,
        request: Request<InitSceneRequest>,
    ) -> Result<Response<Empty>, Status> {
        let req = request.get_ref();
        if let Ok(mut scene_tree) = self.scene_tree.lock() {
            /* clear Scene Tree */
            scene_tree.clear();

            /* create ground */
            let ground = geometry::Ground::new(20.0);
            scene_tree.set_geometry("/ground", Box::new(ground));
            scene_tree.set_transform(
                "/ground",
                na::Isometry3::from_parts(
                    na::Translation3::new(0.0, 0.0, -1e-4), // prevent z-fitting
                    na::UnitQuaternion::identity(),
                ),
            );
            scene_tree.set_color("/ground", Some((200, 200, 200)));

            /* create container */
            let container_config = &req.container_config;
            if container_config.mesh_filepath.is_some() {
                warn!("Mesh file path is not supported yet.");
            }
            if container_config.thickness.is_some() {
                warn!("Thickness is not supported yet.");
            }
            if container_config.wave_step.is_some() {
                warn!("Wave step is not supported yet.");
            }
            if container_config.wave_height.is_some() {
                warn!("Wave height is not supported yet.");
            }
            let container_size = (
                container_config.size.length,
                container_config.size.width,
                container_config.size.height,
            );
            let container =
                geometry::Container::new(container_size.0, container_size.1, container_size.2);
            scene_tree.set_geometry("/container", Box::new(container));
            scene_tree.set_color("/container", Some((84, 137, 191)));

            /* create carton packing */
            if let Some(packing_config) = &req.packing_config {
                let mut size_list = packing_config
                    .size_list
                    .iter()
                    .map(|s| (s.length, s.width, s.height))
                    .collect::<Vec<_>>();
                let mut pose_list = packing_config
                    .pose_list
                    .iter()
                    .map(|p| {
                        na::Isometry3::from_parts(
                            na::Translation3::new(p.position.x, p.position.y, p.position.z),
                            na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                                p.rotation.w,
                                p.rotation.x,
                                p.rotation.y,
                                p.rotation.z,
                            )),
                        )
                    })
                    .collect::<Vec<_>>();
                if !pose_list.is_empty() && size_list.len() != pose_list.len() {
                    warn!("Size list and pose list have different lengths.");
                    let min_len = size_list.len().min(pose_list.len());
                    size_list.truncate(min_len);
                    pose_list.truncate(min_len);
                }

                /* bin packing */
                if pose_list.is_empty() && packing_config.bin_packing_algorithm.is_none() {
                    return Err(Status::invalid_argument(
                        "pose_list and bin_packing_algorithm are both empty.",
                    ));
                }
                if let Some(bin_packing_algorithm) = &packing_config.bin_packing_algorithm {
                    let bin_size = bin_packing_algorithm
                        .bin_size
                        .unwrap_or(container_config.size);
                    if let Ok(method) = BinPackingMethod::try_from(bin_packing_algorithm.method) {
                        match method {
                            BinPackingMethod::SimpleStacking => {
                                pose_list = simple_stacking(
                                    &size_list,
                                    &(bin_size.length, bin_size.width, bin_size.height),
                                );
                            }
                        };
                        if pose_list.len() != size_list.len() {
                            warn!(
                                "Only packed {} out of {} items.",
                                pose_list.len(),
                                size_list.len()
                            );
                            let min_len = size_list.len().min(pose_list.len());
                            size_list.truncate(min_len);
                            pose_list.truncate(min_len);
                        }
                        /* convert to container space */
                        for pose in pose_list.iter_mut() {
                            *pose = na::Isometry3::from_parts(
                                na::Translation3::new(bin_size.length, bin_size.width * 0.5, 0.0),
                                na::UnitQuaternion::from_euler_angles(
                                    0.0,
                                    0.0,
                                    std::f32::consts::PI,
                                ),
                            ) * (*pose);
                        }
                    } else {
                        error!(
                            "Unknown bin packing method: {}",
                            bin_packing_algorithm.method
                        );
                    }
                }

                /* TODO: align carton rotation */

                /* randomization */
                let mut rng = rand::thread_rng();
                if let Some(randomization) = &packing_config.randomization {
                    for size in size_list.iter_mut() {
                        if let Some(length_range) = randomization.length_range {
                            size.0 +=
                                rng.gen_range(fixed_range(length_range.min..length_range.max));
                        }
                        if let Some(width_range) = randomization.width_range {
                            size.1 += rng.gen_range(fixed_range(width_range.min..width_range.max));
                        }
                        if let Some(height_range) = randomization.height_range {
                            size.2 +=
                                rng.gen_range(fixed_range(height_range.min..height_range.max));
                        }
                    }
                    for pose in pose_list.iter_mut() {
                        let mut translation = (0.0f32, 0.0f32, 0.0f32);
                        let mut euler_angles = (0.0f32, 0.0f32, 0.0f32);
                        if let Some(x_range) = randomization.x_range {
                            translation.0 = rng.gen_range(fixed_range(x_range.min..x_range.max));
                        }
                        if let Some(y_range) = randomization.y_range {
                            translation.1 = rng.gen_range(fixed_range(y_range.min..y_range.max));
                        }
                        if let Some(z_range) = randomization.z_range {
                            translation.2 = rng.gen_range(fixed_range(z_range.min..z_range.max));
                        }
                        if let Some(euler_x_range) = randomization.euler_x_range {
                            euler_angles.0 =
                                rng.gen_range(fixed_range(euler_x_range.min..euler_x_range.max));
                        }
                        if let Some(euler_y_range) = randomization.euler_y_range {
                            euler_angles.1 =
                                rng.gen_range(fixed_range(euler_y_range.min..euler_y_range.max));
                        }
                        if let Some(euler_z_range) = randomization.euler_z_range {
                            euler_angles.2 =
                                rng.gen_range(fixed_range(euler_z_range.min..euler_z_range.max));
                        }
                        *pose = (*pose)
                            * na::Isometry3::from_parts(
                                na::Translation3::new(translation.0, translation.1, translation.2),
                                na::UnitQuaternion::from_euler_angles(
                                    euler_angles.0,
                                    euler_angles.1,
                                    euler_angles.2,
                                ),
                            );
                    }
                }

                /* physics */
                if let Some(physics) = &packing_config.physics {
                    /* align to container */
                    let mut aabb = AABB::new();
                    for (size, pose) in size_list.iter().zip(pose_list.iter()) {
                        aabb.insert_cuboid(&(size.0 * 0.5, size.1 * 0.5, size.2 * 0.5), &pose);
                    }
                    let translation = na::Translation3::new(
                        container_size.0 - aabb.max.0,
                        -(aabb.max.1 + aabb.min.1) * 0.5,
                        -aabb.min.2,
                    );
                    for pose in pose_list.iter_mut() {
                        *pose =
                            na::Isometry3::from_parts(translation, na::UnitQuaternion::identity())
                                * (*pose);
                    }

                    /* free fall */
                    pose_list = bake_cuboids_by_free_fall(
                        &size_list
                            .iter()
                            .map(|s| (s.0 * 0.5, s.1 * 0.5, s.2 * 0.5))
                            .collect::<Vec<_>>(),
                        &pose_list,
                        &container_size,
                        physics.drop_height,
                        physics.num_steps,
                    );

                    /* remove boxes outside */
                    let mut indices = Vec::new();
                    const MARGIN: f32 = 0.02;
                    for (i, (size, pose)) in size_list.iter().zip(pose_list.iter()).enumerate() {
                        let half_size = (size.0 * 0.5, size.1 * 0.5, size.2 * 0.5);
                        let vertices = vec![
                            (-half_size.0, -half_size.1, -half_size.2),
                            (half_size.0, -half_size.1, -half_size.2),
                            (half_size.0, half_size.1, -half_size.2),
                            (-half_size.0, half_size.1, -half_size.2),
                            (-half_size.0, -half_size.1, half_size.2),
                            (half_size.0, -half_size.1, half_size.2),
                            (half_size.0, half_size.1, half_size.2),
                            (-half_size.0, half_size.1, half_size.2),
                        ];
                        let mut outside = false;
                        for v in vertices.iter() {
                            let p = pose * na::Point3::new(v.0, v.1, v.2);
                            if p.x < 0.0 - MARGIN
                                || p.x > container_size.0 + MARGIN
                                || p.y < -container_size.1 * 0.5 - MARGIN
                                || p.y > container_size.1 * 0.5 + MARGIN
                                || p.z < 0.0 - MARGIN
                                || p.z > container_size.2 + MARGIN
                            {
                                outside = true;
                                break;
                            }
                        }
                        if !outside {
                            indices.push(i);
                        }
                    }
                    info!(
                        "{} out of {} boxes are inside the container.",
                        indices.len(),
                        size_list.len()
                    );
                    pose_list = indices.iter().map(|&i| pose_list[i]).collect();
                    size_list = indices.iter().map(|&i| size_list[i]).collect();

                    /* TODO: shrink boxes */
                }

                for (i, (size, pose)) in size_list.iter().zip(pose_list.iter()).enumerate() {
                    let carton = geometry::Carton::new(size.0, size.1, size.2);
                    scene_tree.set_geometry(&format!("/carton/{}", i), Box::new(carton));
                    scene_tree.set_transform(&format!("/carton/{}", i), pose.clone());
                    scene_tree.set_color(&format!("/carton/{}", i), Some((211, 186, 156)));
                }
            } else {
                info!("No packing config is provided. Empty container.");
            }

            // publish to Rerun
            if let Some(rec) = self.rec.as_ref() {
                let _ = rec.log(
                    "/",
                    &rerun::Clear {
                        is_recursive: ClearIsRecursive(true),
                    },
                );
                for (path, node) in scene_tree.iter() {
                    if node.transform.is_some() && node.geometry.is_some() {
                        let tf = node.transform.as_ref().unwrap();
                        let geom = node.geometry.as_ref().unwrap();
                        let color = node.color;
                        let vertices = geom.vertices();
                        let vertices = vertices
                            .iter()
                            .map(|v| {
                                let p = tf * na::Point3::new(v.0, v.1, v.2);
                                (p.x, p.y, p.z)
                            })
                            .collect::<Vec<_>>();
                        let _ = rec.log(
                            path.as_str(),
                            &to_rerun_mesh3d(&vertices, &geom.indices()).with_vertex_colors(color),
                        );
                    }
                }
            }
        }
        let reply = Empty {};
        Ok(Response::new(reply))
    }

    async fn init_robot(
        &self,
        request: Request<InitRobotRequest>,
    ) -> Result<Response<Empty>, Status> {
        /* Test code */
        if let Ok(mut scene_tree) = self.scene_tree.lock() {
            for i in 0..7 {
                let link = geometry::TriMesh::from_stl(
                    format!(
                        "/home/shq/Projects/mycode/resolve_collision/data/fanuc/mesh/link{}.stl",
                        i
                    )
                    .as_str(),
                );
                scene_tree.set_geometry(format!("/robot/link{}", i).as_str(), Box::new(link));
                scene_tree.set_color(format!("/robot/link{}", i).as_str(), Some((247, 202, 1)));
            }
            // publish to Rerun
            if let Some(rec) = self.rec.as_ref() {
                for (path, node) in scene_tree.iter() {
                    if node.transform.is_some() && node.geometry.is_some() {
                        let tf = node.transform.as_ref().unwrap();
                        let geom = node.geometry.as_ref().unwrap();
                        let color = node.color;
                        let vertices = geom.vertices();
                        let vertices = vertices
                            .iter()
                            .map(|v| {
                                let p = tf * na::Point3::new(v.0, v.1, v.2);
                                (p.x, p.y, p.z)
                            })
                            .collect::<Vec<_>>();
                        let _ = rec.log(
                            path.as_str(),
                            &to_rerun_mesh3d(&vertices, &geom.indices()).with_vertex_colors(color),
                        );
                    }
                }
            }
        }
        let reply = Empty {};
        Ok(Response::new(reply))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let addr = "[::1]:52234".parse()?;
    let sim_world = MySimWorld::new();

    Server::builder()
        .add_service(SimWorldServer::new(sim_world))
        .serve(addr)
        .await?;

    Ok(())
}
