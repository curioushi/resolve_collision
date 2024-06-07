use ncollide3d::na;
use ncollide3d::shape::Cuboid;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct CubeSerde {
    pub tf: Vec<Vec<f64>>,
    pub size: Vec<f64>,
}

impl CubeSerde {
    pub fn from_cube(c: &CuboidWithTf) -> Self {
        let tf = c.tf.to_homogeneous();
        let size = c.cuboid.half_extents * 2.0;
        CubeSerde {
            tf: tf
                .row_iter()
                .map(|row| row.iter().map(|x| *x).collect())
                .collect(),
            size: size.iter().map(|x| *x).collect(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct CuboidWithTf {
    pub cuboid: Cuboid<f64>,
    pub tf: na::Isometry3<f64>,
}

impl CuboidWithTf {
    pub fn from_cube_serde(c: &CubeSerde) -> Self {
        let rotmat = na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[
            na::Vector3::new(c.tf[0][0], c.tf[1][0], c.tf[2][0]),
            na::Vector3::new(c.tf[0][1], c.tf[1][1], c.tf[2][1]),
            na::Vector3::new(c.tf[0][2], c.tf[1][2], c.tf[2][2]),
        ]));
        let translation = na::Translation3::new(c.tf[0][3], c.tf[1][3], c.tf[2][3]);
        let tf = na::Isometry3::from_parts(translation, rotmat.into());
        let size = na::Vector3::new(c.size[0], c.size[1], c.size[2]);
        CuboidWithTf {
            cuboid: Cuboid::new(size / 2.0),
            tf,
        }
    }

    pub fn rot_x_90(&mut self) {
        let half_extents = self.cuboid.half_extents;
        self.cuboid = Cuboid::new(na::Vector3::new(
            half_extents[0],
            half_extents[2],
            half_extents[1],
        ));
        self.tf.rotation = self.tf.rotation
            * na::Rotation3::from_axis_angle(&na::Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
    }

    pub fn rot_y_90(&mut self) {
        let half_extents = self.cuboid.half_extents;
        self.cuboid = Cuboid::new(na::Vector3::new(
            half_extents[2],
            half_extents[1],
            half_extents[0],
        ));
        self.tf.rotation = self.tf.rotation
            * na::Rotation3::from_axis_angle(&na::Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
    }

    pub fn rot_z_90(&mut self) {
        let half_extents = self.cuboid.half_extents;
        self.cuboid = Cuboid::new(na::Vector3::new(
            half_extents[1],
            half_extents[0],
            half_extents[2],
        ));
        self.tf.rotation = self.tf.rotation
            * na::Rotation3::from_axis_angle(&na::Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
    }

    pub fn new_rot_x_90(&self) -> CuboidWithTf {
        let mut cube = self.clone();
        cube.rot_x_90();
        cube
    }

    pub fn new_rot_y_90(&self) -> CuboidWithTf {
        let mut cube = self.clone();
        cube.rot_y_90();
        cube
    }

    pub fn new_rot_z_90(&self) -> CuboidWithTf {
        let mut cube = self.clone();
        cube.rot_z_90();
        cube
    }
}
