use ncollide3d::shape::Cuboid;
use ncollide3d::{na, simba::scalar::SubsetOf};
use serde::{Deserialize, Serialize};
use std::ops::{Deref, DerefMut};

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

#[derive(Debug, Clone)]
pub struct Isometry3Serde(na::Isometry3<f64>);

impl Isometry3Serde {
    pub fn new(tf: na::Isometry3<f64>) -> Self {
        Isometry3Serde(tf)
    }
}

impl Deref for Isometry3Serde {
    type Target = na::Isometry3<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Isometry3Serde {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl AsRef<na::Isometry3<f64>> for Isometry3Serde {
    fn as_ref(&self) -> &na::Isometry3<f64> {
        &self.0
    }
}

impl AsMut<na::Isometry3<f64>> for Isometry3Serde {
    fn as_mut(&mut self) -> &mut na::Isometry3<f64> {
        &mut self.0
    }
}

impl Serialize for Isometry3Serde {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let m = self.0.to_homogeneous();
        let data = [
            [m[(0, 0)], m[(0, 1)], m[(0, 2)], m[(0, 3)]],
            [m[(1, 0)], m[(1, 1)], m[(1, 2)], m[(1, 3)]],
            [m[(2, 0)], m[(2, 1)], m[(2, 2)], m[(2, 3)]],
            [m[(3, 0)], m[(3, 1)], m[(3, 2)], m[(3, 3)]],
        ];
        data.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Isometry3Serde {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct SerdeData([[f64; 4]; 4]);

        let helper = SerdeData::deserialize(deserializer)?;
        let data = helper.0;

        let m = na::Matrix4::new(
            data[0][0], data[0][1], data[0][2], data[0][3], data[1][0], data[1][1], data[1][2],
            data[1][3], data[2][0], data[2][1], data[2][2], data[2][3], data[3][0], data[3][1],
            data[3][2], data[3][3],
        );

        Ok(Isometry3Serde(na::Isometry3::from_superset_unchecked(&m)))
    }
}

#[derive(Debug, Clone)]
pub struct CuboidSerde(Cuboid<f64>);

impl Deref for CuboidSerde {
    type Target = Cuboid<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for CuboidSerde {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl AsRef<Cuboid<f64>> for CuboidSerde {
    fn as_ref(&self) -> &Cuboid<f64> {
        &self.0
    }
}

impl AsMut<Cuboid<f64>> for CuboidSerde {
    fn as_mut(&mut self) -> &mut Cuboid<f64> {
        &mut self.0
    }
}

impl Serialize for CuboidSerde {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let data = [
            self.0.half_extents[0] * 2.0,
            self.0.half_extents[1] * 2.0,
            self.0.half_extents[2] * 2.0,
        ];
        data.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for CuboidSerde {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct SerdeData([f64; 3]);

        let helper = SerdeData::deserialize(deserializer)?;
        let data = helper.0;

        Ok(CuboidSerde::new(na::Vector3::new(
            data[0] * 0.5,
            data[1] * 0.5,
            data[2] * 0.5,
        )))
    }
}

impl CuboidSerde {
    pub fn new(half_extents: na::Vector3<f64>) -> Self {
        CuboidSerde(Cuboid::new(half_extents))
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CuboidWithTf {
    pub cuboid: CuboidSerde,
    pub tf: Isometry3Serde,
}

impl CuboidWithTf {
    pub fn rot_x_90(&mut self) {
        let half_extents = self.cuboid.half_extents;
        self.cuboid = CuboidSerde::new(na::Vector3::new(
            half_extents[0],
            half_extents[2],
            half_extents[1],
        ));
        self.tf.rotation = self.tf.rotation
            * na::Rotation3::from_axis_angle(&na::Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
    }

    pub fn rot_y_90(&mut self) {
        let half_extents = self.cuboid.half_extents;
        self.cuboid = CuboidSerde::new(na::Vector3::new(
            half_extents[2],
            half_extents[1],
            half_extents[0],
        ));
        self.tf.rotation = self.tf.rotation
            * na::Rotation3::from_axis_angle(&na::Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
    }

    pub fn rot_z_90(&mut self) {
        let half_extents = self.cuboid.half_extents;
        self.cuboid = CuboidSerde::new(na::Vector3::new(
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
