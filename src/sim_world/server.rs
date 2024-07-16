use rand::Rng;
use tonic::{transport::Server, Request, Response, Status};

use ncollide3d::na;
use sim_world::sim_world_server::{SimWorld, SimWorldServer};
use sim_world::{EmptyRequest, NewSceneResponse};

mod geometry;
use geometry::{to_rerun_mesh3d, SceneTree};

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
            for i in 0..10 {
                let path = format!("/cartons/{}", i);
                let hsize_x = rng.gen_range(0.1f32..1.0f32);
                let hsize_y = rng.gen_range(0.1f32..1.0f32);
                let hsize_z = rng.gen_range(0.1f32..1.0f32);
                scene_tree.set_geometry(
                    path.as_str(),
                    Box::new(geometry::Carton::new(hsize_x, hsize_y, hsize_z)),
                );
                let x = rng.gen_range(-5.0f32..5.0f32);
                let y = rng.gen_range(-5.0f32..5.0f32);
                let z = rng.gen_range(-5.0f32..5.0f32);
                let euler_x = rng.gen_range(0.0f32..std::f32::consts::PI);
                let euler_y = rng.gen_range(0.0f32..std::f32::consts::PI);
                let euler_z = rng.gen_range(0.0f32..std::f32::consts::PI);
                scene_tree.set_transform(
                    path.as_str(),
                    na::Isometry3::new(
                        na::Vector3::new(x, y, z),
                        na::Vector3::new(euler_x, euler_y, euler_z),
                    ),
                );
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
                                .with_vertex_colors(Some(rerun::Color::from_rgb(0, 255, 0))),
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
