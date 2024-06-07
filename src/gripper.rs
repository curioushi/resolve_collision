use geo_types::{coord, LineString, Polygon};
use ncollide3d::na;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct SuctionGroupSerde {
    pub shape: Vec<(f64, f64)>,
}

#[derive(Serialize, Deserialize)]
pub struct GripperSerde {
    pub tf_flange_tip: Vec<Vec<f64>>,
    pub suction_groups: Vec<SuctionGroupSerde>,
}

impl GripperSerde {
    pub fn from_gripper(gripper: &Gripper) -> Self {
        let tf_flange_tip = gripper
            .tf_flange_tip
            .to_homogeneous()
            .row_iter()
            .map(|row| row.iter().map(|x| *x).collect())
            .collect();

        let mut suction_groups = vec![];
        for sg in gripper.suction_groups.iter() {
            let mut shape = vec![];
            for p in sg.shape.exterior() {
                shape.push((p.x, p.y));
            }
            suction_groups.push(SuctionGroupSerde { shape });
        }

        Self {
            tf_flange_tip,
            suction_groups,
        }
    }

    pub fn to_gripper(&self) -> Gripper {
        let tf_flange_tip = na::Isometry3::from_parts(
            na::Translation3::new(
                self.tf_flange_tip[0][3],
                self.tf_flange_tip[1][3],
                self.tf_flange_tip[2][3],
            ),
            na::UnitQuaternion::from_matrix(&na::Matrix3::from_columns(&[
                na::Vector3::new(
                    self.tf_flange_tip[0][0],
                    self.tf_flange_tip[1][0],
                    self.tf_flange_tip[2][0],
                ),
                na::Vector3::new(
                    self.tf_flange_tip[0][1],
                    self.tf_flange_tip[1][1],
                    self.tf_flange_tip[2][1],
                ),
                na::Vector3::new(
                    self.tf_flange_tip[0][2],
                    self.tf_flange_tip[1][2],
                    self.tf_flange_tip[2][2],
                ),
            ])),
        );

        let mut suction_groups = vec![];
        for sg in self.suction_groups.iter() {
            let mut points = vec![];
            for p in sg.shape.iter() {
                points.push(coord! {x: p.0, y: p.1});
            }
            let shape = Polygon::new(LineString::new(points), vec![]);
            suction_groups.push(SuctionGroup { shape });
        }

        Gripper {
            tf_flange_tip,
            suction_groups,
        }
    }
}

pub struct SuctionGroup {
    pub shape: Polygon,
}

pub struct Gripper {
    pub tf_flange_tip: na::Isometry3<f64>,
    pub suction_groups: Vec<SuctionGroup>,
}
