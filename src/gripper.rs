use crate::common::{isometry_to_vec, vec_to_isometry};
use geo::algorithm::{Area, BooleanOps, Translate};
use geo_types::{coord, LineString, Polygon, Rect};
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
    pub max_payload: f64,
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
            max_payload: gripper.max_payload,
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
            max_payload: self.max_payload,
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct PickPlanSerde {
    pub score: f64,
    pub tf_world_tip: Vec<Vec<f64>>,
    pub tf_world_flange: Vec<Vec<f64>>,
    pub suction_group_mask: Vec<bool>,
    pub main_box_index: usize,
    pub other_box_indices: Vec<usize>,
}

impl PickPlanSerde {
    pub fn from_pick_plan(plan: &PickPlan) -> Self {
        Self {
            score: plan.score,
            tf_world_tip: isometry_to_vec(&plan.tf_world_tip),
            tf_world_flange: isometry_to_vec(&plan.tf_world_flange),
            suction_group_mask: plan.suction_group_mask.clone(),
            main_box_index: plan.main_box_index,
            other_box_indices: plan.other_box_indices.clone(),
        }
    }

    pub fn to_pick_plan(&self) -> PickPlan {
        PickPlan {
            score: self.score,
            tf_world_tip: vec_to_isometry(&self.tf_world_tip),
            tf_world_flange: vec_to_isometry(&self.tf_world_flange),
            suction_group_mask: self.suction_group_mask.clone(),
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
    // pub collision_shape: Compound<f64>,
    pub max_payload: f64,
}

impl Gripper {
    pub fn suction_areas(&self) -> Vec<f64> {
        let areas: Vec<f64> = self
            .suction_groups
            .iter()
            .map(|sg| sg.shape.unsigned_area())
            .collect();
        areas
    }
    pub fn suction_rect(&self) -> Rect {
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        for sg in self.suction_groups.iter() {
            for p in sg.shape.exterior() {
                min_x = min_x.min(p.x);
                min_y = min_y.min(p.y);
                max_x = max_x.max(p.x);
                max_y = max_y.max(p.y);
            }
        }
        Rect::new(coord! {x: min_x, y: min_y}, coord! {x: max_x, y: max_y})
    }
    pub fn intersection_areas(&self, polygon: &Polygon, dx: f64, dy: f64) -> Vec<f64> {
        let polygon = polygon.translate(-dx, -dy); // multiply by -1 for gripper translation
        let areas: Vec<f64> = self
            .suction_groups
            .iter()
            .map(|sg| sg.shape.intersection(&polygon).unsigned_area())
            .collect();
        areas
    }
}

#[derive(Debug)]
pub struct PickPlan {
    pub score: f64,
    pub tf_world_tip: na::Isometry3<f64>,
    pub tf_world_flange: na::Isometry3<f64>,
    pub suction_group_mask: Vec<bool>,
    pub main_box_index: usize,
    pub other_box_indices: Vec<usize>,
}
