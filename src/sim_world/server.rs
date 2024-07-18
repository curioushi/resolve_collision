use tonic::{transport::Server, Request, Response, Status};

use ncollide3d::na;
use sim_world::sim_world_server::{SimWorld, SimWorldServer};
use sim_world::{Empty, NewSceneRequest};

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
        let rec = if let Ok(recording) = rerun::RecordingStreamBuilder::new("sim_world").spawn() {
            Some(recording)
        } else {
            None
        };
        Self { rec, scene_tree }
    }
}

#[tonic::async_trait]
impl SimWorld for MySimWorld {
    async fn new_scene(
        &self,
        request: Request<NewSceneRequest>,
    ) -> Result<Response<Empty>, Status> {
        let req = request.get_ref();
        if let Ok(mut scene_tree) = self.scene_tree.lock() {
            scene_tree.clear();
            let container_config = &req.container_config;
            let container = geometry::Container::new(
                container_config.size.length,
                container_config.size.width,
                container_config.size.height,
            );
            scene_tree.set_geometry("/container", Box::new(container));
            scene_tree.set_transform("/container", na::Isometry3::identity());
            // publish to Rerun
            if let Some(rec) = self.rec.as_ref() {
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
        let reply = Empty {};
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
