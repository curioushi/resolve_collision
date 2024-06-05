use ncollide3d::na;
use ncollide3d::shape::Cuboid;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct CubeSerde {
    pub tf: Vec<Vec<f64>>,
    pub size: Vec<f64>,
}

#[derive(Debug)]
pub struct CuboidWithTf {
    pub cuboid: Cuboid<f64>,
    pub tf: na::Isometry3<f64>,
}

impl CuboidWithTf {
    pub fn from_cube_serde(c: &CubeSerde) -> Self {
        let rotmat = na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[
            na::Vector3::new(c.tf[0][0], c.tf[0][1], c.tf[0][2]),
            na::Vector3::new(c.tf[1][0], c.tf[1][1], c.tf[1][2]),
            na::Vector3::new(c.tf[2][0], c.tf[2][1], c.tf[2][2]),
        ]));
        let translation = na::Translation3::new(c.tf[0][3], c.tf[1][3], c.tf[2][3]);
        let tf = na::Isometry3::from_parts(translation, rotmat.into());
        let size = na::Vector3::new(c.size[0], c.size[1], c.size[2]);
        CuboidWithTf {
            cuboid: Cuboid::new(size / 2.0),
            tf,
        }
    }
}
