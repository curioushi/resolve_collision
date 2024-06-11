use crate::common::{isometry_to_vec, vec_to_isometry};
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
        let mut suction_groups = vec![];
        for sg in gripper.suction_groups.iter() {
            let mut shape = vec![];
            for p in sg.shape.exterior() {
                shape.push((p.x, p.y));
            }
            suction_groups.push(SuctionGroupSerde { shape });
        }

        Self {
            tf_flange_tip: isometry_to_vec(&gripper.tf_flange_tip),
            suction_groups,
        }
    }

    pub fn to_gripper(&self) -> Gripper {
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
            tf_flange_tip: vec_to_isometry(&self.tf_flange_tip),
            suction_groups,
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct PickPlanSerde {
    pub score: f64,
    pub tf_world_tip: Vec<Vec<f64>>,
    pub tf_world_flange: Vec<Vec<f64>>,
    pub suction_group_indices: Vec<usize>,
    pub main_box_index: usize,
    pub other_box_indices: Vec<usize>,
}

impl PickPlanSerde {
    pub fn from_pick_plan(plan: &PickPlan) -> Self {
        Self {
            score: plan.score,
            tf_world_tip: isometry_to_vec(&plan.tf_world_tip),
            tf_world_flange: isometry_to_vec(&plan.tf_world_flange),
            suction_group_indices: plan.suction_group_indices.clone(),
            main_box_index: plan.main_box_index,
            other_box_indices: plan.other_box_indices.clone(),
        }
    }

    pub fn to_pick_plan(&self) -> PickPlan {
        PickPlan {
            score: self.score,
            tf_world_tip: vec_to_isometry(&self.tf_world_tip),
            tf_world_flange: vec_to_isometry(&self.tf_world_flange),
            suction_group_indices: self.suction_group_indices.clone(),
            main_box_index: self.main_box_index,
            other_box_indices: self.other_box_indices.clone(),
        }
    }
}

#[derive(Debug)]
pub struct SuctionGroup {
    pub shape: Polygon,
}

#[derive(Debug)]
pub struct Gripper {
    pub tf_flange_tip: na::Isometry3<f64>,
    pub suction_groups: Vec<SuctionGroup>,
}

#[derive(Debug)]
pub struct PickPlan {
    pub score: f64,
    pub tf_world_tip: na::Isometry3<f64>,
    pub tf_world_flange: na::Isometry3<f64>,
    pub suction_group_indices: Vec<usize>,
    pub main_box_index: usize,
    pub other_box_indices: Vec<usize>,
}
