use rand::Rng;
use tonic::{transport::Server, Request, Response, Status};

use ncollide3d::na;
use sim_world::sim_world_server::{SimWorld, SimWorldServer};
use sim_world::{EmptyRequest, NewSceneResponse};

mod geometry;
use geometry::{bake_cuboids_by_free_fall, to_rerun_mesh3d, GeometryType, SceneTree};

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
        let mut rng = rand::thread_rng();
        if let Ok(mut scene_tree) = scene_tree.lock() {
            let ground = geometry::Ground::new(30.0f32);
            scene_tree.set_geometry("/ground", Box::new(ground));
            scene_tree.set_transform(
                "/ground",
                na::Isometry3::from_parts(
                    na::Translation3::new(0.0f32, 0.0f32, -0.0001f32), // prevent z-fighting
                    na::UnitQuaternion::identity(),
                ),
            );
            let container = geometry::Container::new(8.0f32, 3.0f32, 3.0f32);
            scene_tree.set_geometry("/container", Box::new(container));
            scene_tree.set_transform("/container", na::Isometry3::identity());
            let mut half_sizes = Vec::new();
            let mut initial_poses = Vec::new();
            let num = 100;
            for i in 0..num {
                let path = format!("/cartons/{}", i);
                let size_x = rng.gen_range(0.2f32..0.8f32);
                let size_y = rng.gen_range(0.2f32..0.8f32);
                let size_z = rng.gen_range(0.2f32..0.8f32);
                scene_tree.set_geometry(
                    path.as_str(),
                    Box::new(geometry::Carton::new(size_x, size_y, size_z)),
                );
                let x = rng.gen_range(-2.0f32..2.0f32);
                let y = rng.gen_range(-2.0f32..2.0f32);
                let z = rng.gen_range(5.0f32..10.0f32);
                let euler_x = rng.gen_range(0.0f32..std::f32::consts::PI);
                let euler_y = rng.gen_range(0.0f32..std::f32::consts::PI);
                let euler_z = rng.gen_range(0.0f32..std::f32::consts::PI);
                let pose = na::Isometry3::new(
                    na::Vector3::new(x, y, z),
                    na::Vector3::new(euler_x, euler_y, euler_z),
                );
                scene_tree.set_transform(path.as_str(), pose);
                half_sizes.push((size_x * 0.5, size_y * 0.5, size_z * 0.5));
                initial_poses.push(pose);
            }
            let final_poses = bake_cuboids_by_free_fall(&half_sizes, &initial_poses);
            for i in 0..num {
                let path = format!("/cartons/{}", i);
                scene_tree.set_transform(path.as_str(), final_poses[i]);
            }
        }
        if let Ok(recording) = rerun::RecordingStreamBuilder::new("sim_world").spawn() {
            Self {
                rec: Some(recording),
                scene_tree,
            }
        } else {
            Self {
                rec: None,
                scene_tree,
            }
        }
    }
}

#[tonic::async_trait]
impl SimWorld for MySimWorld {
    async fn new_scene(
        &self,
        _request: Request<EmptyRequest>,
    ) -> Result<Response<NewSceneResponse>, Status> {
        let reply = NewSceneResponse {
            status: Some(sim_world::Status {
                error_code: 0,
                message: "Success".into(),
            }),
        };

        if let Some(rec) = self.rec.as_ref() {
            if let Ok(scene_tree) = self.scene_tree.lock() {
                for (path, node) in scene_tree.iter() {
                    if node.transform.is_some() && node.geometry.is_some() {
                        let tf = node.transform.as_ref().unwrap();
                        let geom = node.geometry.as_ref().unwrap();
                        let color = match geom.type_id() {
                            GeometryType::Ground => rerun::Color::from_rgb(200, 200, 200),
                            GeometryType::Carton => rerun::Color::from_rgb(211, 186, 156),
                            GeometryType::Container => rerun::Color::from_rgb(84, 137, 191),
                        };
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
                            &to_rerun_mesh3d(&vertices, &geom.indices())
                                .with_vertex_colors(Some(color)),
                        );
                    }
                }
            }
        }
        Ok(Response::new(reply))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr = "[::1]:52234".parse()?;
    let sim_world = MySimWorld::new();

    Server::builder()
        .add_service(SimWorldServer::new(sim_world))
        .serve(addr)
        .await?;

    Ok(())
}
