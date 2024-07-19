use sim_world::sim_world_client::SimWorldClient;
use sim_world::{
    BinPackingAlgorithm, BinPackingMethod, BoxRandomization, GenContainerConfig, GenPackingConfig,
    InitRobotRequest, InitSceneRequest, PackingPhysics, Range, Size,
};

pub mod sim_world {
    tonic::include_proto!("sim_world");
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = SimWorldClient::connect("http://[::1]:52234").await?;

    let size_list = (0..1000)
        .into_iter()
        .map(|_| Size {
            length: 0.75,
            width: 0.45,
            height: 0.45,
        })
        .collect::<Vec<_>>();

    let randomization = Some(BoxRandomization {
        length_range: Some(Range {
            min: -0.04,
            max: 0.04,
        }),
        width_range: Some(Range {
            min: -0.04,
            max: 0.04,
        }),
        height_range: Some(Range {
            min: -0.04,
            max: 0.04,
        }),
        x_range: Some(Range {
            min: -0.1,
            max: 0.1,
        }),
        y_range: Some(Range {
            min: -0.1,
            max: 0.1,
        }),
        z_range: Some(Range {
            min: -0.1,
            max: 0.1,
        }),
        euler_x_range: Some(Range {
            min: -0.05,
            max: 0.05,
        }),
        euler_y_range: Some(Range {
            min: -0.05,
            max: 0.05,
        }),
        euler_z_range: Some(Range {
            min: -0.05,
            max: 0.05,
        }),
    });

    let physics = Some(PackingPhysics {
        drop_height: Some(1.0),
        num_steps: Some(400),
    });

    let request = tonic::Request::new(InitSceneRequest {
        container_config: GenContainerConfig {
            size: Size {
                length: 12.0,
                width: 3.0,
                height: 3.0,
            },
            mesh_filepath: None,
            thickness: None,
            wave_step: None,
            wave_height: None,
        },
        packing_config: Some(GenPackingConfig {
            size_list,
            pose_list: Vec::new(),
            bin_packing_algorithm: Some(BinPackingAlgorithm {
                method: BinPackingMethod::SimpleStacking.into(),
                bin_size: Some(Size {
                    length: 8.0,
                    width: 3.0,
                    height: 3.0,
                }),
            }),
            randomization: None,
            physics,
        }),
    });

    let response = client.init_scene(request).await?;
    println!("RESPONSE={:?}", response);

    let request = tonic::Request::new(InitRobotRequest {});
    let response = client.init_robot(request).await?;
    println!("RESPONSE={:?}", response);

    Ok(())
}
