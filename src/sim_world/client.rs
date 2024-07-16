use sim_world::sim_world_client::SimWorldClient;
use sim_world::EmptyRequest;

pub mod sim_world {
    tonic::include_proto!("sim_world");
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = SimWorldClient::connect("http://[::1]:52234").await?;

    let request = tonic::Request::new(EmptyRequest {});

    let response = client.new_scene(request).await?;

    println!("RESPONSE={:?}", response);

    Ok(())
}
