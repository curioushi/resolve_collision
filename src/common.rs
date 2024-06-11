use ncollide3d::na;
use ncollide3d::shape::Cuboid;
use serde::{Deserialize, Serialize};

pub fn isometry_to_vec(isometry: &na::Isometry3<f64>) -> Vec<Vec<f64>> {
    let tf = isometry.to_homogeneous();
    tf.row_iter()
        .map(|row| row.iter().map(|x| *x).collect())
        .collect()
}

pub fn vec_to_isometry(vec: &Vec<Vec<f64>>) -> na::Isometry3<f64> {
    na::Isometry3::from_parts(
        na::Translation3::new(vec[0][3], vec[1][3], vec[2][3]),
        na::UnitQuaternion::from_matrix(&na::Matrix3::from_columns(&[
            na::Vector3::new(vec[0][0], vec[1][0], vec[2][0]),
            na::Vector3::new(vec[0][1], vec[1][1], vec[2][1]),
            na::Vector3::new(vec[0][2], vec[1][2], vec[2][2]),
        ])),
    )
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CubeSerde {
    pub tf: Vec<Vec<f64>>,
    pub size: Vec<f64>,
}

impl CubeSerde {
    pub fn from_cube(c: &CuboidWithTf) -> Self {
        let size = c.cuboid.half_extents * 2.0;
        CubeSerde {
            tf: isometry_to_vec(&c.tf),
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
        let size = na::Vector3::new(c.size[0], c.size[1], c.size[2]);
        CuboidWithTf {
            cuboid: Cuboid::new(size / 2.0),
            tf: vec_to_isometry(&c.tf),
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
