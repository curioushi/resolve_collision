use sim_world::sim_world_client::SimWorldClient;
use sim_world::{GenContainerConfig, NewSceneRequest, Size};

pub mod sim_world {
    tonic::include_proto!("sim_world");
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = SimWorldClient::connect("http://[::1]:52234").await?;

    let request = tonic::Request::new(NewSceneRequest {
        container_config: GenContainerConfig {
            size: Size {
                length: 8.0,
                width: 3.0,
                height: 3.0,
            },
            mesh_filepath: None,
            thickness: None,
            wave_step: None,
            wave_height: None,
        },
        packing_config: None,
    });

    let response = client.new_scene(request).await?;

    println!("RESPONSE={:?}", response);

    Ok(())
}
