use crate::common::{isometry_to_vec, vec_to_isometry, CuboidSerde, CuboidWithTf, Isometry3Serde};
use geo::algorithm::{Area, Translate};
use geo_types::{coord, Rect};
use ncollide3d::na;
use ncollide3d::shape::{Compound, Cuboid, ShapeHandle};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct SuctionGroupSerde {
    pub shape: Vec<(f64, f64)>,
}

#[derive(Serialize, Deserialize)]
pub struct GripperSerde {
    pub tf_flange_tip: Vec<Vec<f64>>,
    pub suction_groups: Vec<SuctionGroupSerde>,
    pub collision_shape: Vec<CuboidWithTf>,
}

impl GripperSerde {
    pub fn from_gripper(gripper: &Gripper) -> Self {
        let mut suction_groups = vec![];
        for sg in gripper.suction_groups.iter() {
            let mut shape = vec![];
            shape.push((sg.shape.min().x, sg.shape.min().y));
            shape.push((sg.shape.max().x, sg.shape.max().y));
            suction_groups.push(SuctionGroupSerde { shape });
        }
        let mut collision_shape = vec![];
        for (tf, shape) in gripper.collision_shape.shapes().iter() {
            let cuboid = shape.downcast_ref::<Cuboid<f64>>().unwrap();
            collision_shape.push(CuboidWithTf {
                cuboid: CuboidSerde::new(cuboid.half_extents),
                tf: Isometry3Serde::new(tf.clone()),
            });
        }
        Self {
            tf_flange_tip: isometry_to_vec(&gripper.tf_flange_tip),
            suction_groups,
            collision_shape,
        }
    }

    pub fn to_gripper(&self) -> Gripper {
        let mut suction_groups = vec![];
        for sg in self.suction_groups.iter() {
            let mut points = vec![];
            for p in sg.shape.iter() {
                points.push(coord! {x: p.0, y: p.1});
            }
            let shape = Rect::new(points[0], points[1]);
            suction_groups.push(SuctionGroup { shape });
        }
        let mut shapes = vec![];
        for c_with_tf in self.collision_shape.iter() {
            shapes.push((*c_with_tf.tf, ShapeHandle::new(*c_with_tf.cuboid)))
        }
        let collision_shape = Compound::new(shapes);

        Gripper {
            tf_flange_tip: vec_to_isometry(&self.tf_flange_tip),
            suction_groups,
            collision_shape,
        }
    }
}

#[derive(Debug)]
pub struct SuctionGroup {
    pub shape: Rect,
}

pub struct Gripper {
    pub tf_flange_tip: na::Isometry3<f64>,
    pub suction_groups: Vec<SuctionGroup>,
    pub collision_shape: Compound<f64>,
}

fn rect_rect_area(rect1: &Rect, rect2: &Rect) -> f64 {
    let min1 = na::Vector2::new(rect1.min().x, rect1.min().y);
    let min2 = na::Vector2::new(rect2.min().x, rect2.min().y);
    let max1 = na::Vector2::new(rect1.max().x, rect1.max().y);
    let max2 = na::Vector2::new(rect2.max().x, rect2.max().y);
    let min = min1.sup(&min2);
    let max = max1.inf(&max2);
    let diff = (max - min).sup(&na::zero());
    diff.x * diff.y
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
            let mins = sg.shape.min();
            let maxs = sg.shape.max();
            min_x = min_x.min(mins.x);
            min_y = min_y.min(mins.y);
            max_x = max_x.max(maxs.x);
            max_y = max_y.max(maxs.y);
        }
        Rect::new(coord! {x: min_x, y: min_y}, coord! {x: max_x, y: max_y})
    }
    pub fn intersection_areas(&self, rect: &Rect, dx: f64, dy: f64) -> Vec<f64> {
        let rect = rect.translate(-dx, -dy); // multiply by -1 for gripper translation
        let areas: Vec<f64> = self
            .suction_groups
            .iter()
            .map(|sg| rect_rect_area(&sg.shape, &rect))
            .collect();
        areas
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PickPlan {
    pub score: f64,
    pub tf_world_tip: Isometry3Serde,
    pub tf_world_flange: Isometry3Serde,
    pub suction_group_mask: Vec<bool>,
    pub main_box_index: usize,
    pub other_box_indices: Vec<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PickPlanOptions {
    pub max_payload: Option<f64>,
    pub dsafe: Option<f64>,
    pub voxel_size: Option<f64>,
    pub linear_choice: Option<usize>,
    pub max_plans_per_face: Option<usize>,
}

impl Default for PickPlanOptions {
    fn default() -> Self {
        Self {
            max_payload: Some(35.0),
            dsafe: Some(0.0),
            voxel_size: Some(0.4),
            linear_choice: Some(21),
            max_plans_per_face: Some(10000),
        }
    }
}

impl PickPlanOptions {
    pub fn overwrite_from(&mut self, other: &PickPlanOptions) {
        if other.max_payload.is_some() {
            self.max_payload = other.max_payload;
        }
        if other.dsafe.is_some() {
            self.dsafe = other.dsafe;
        }
        if other.voxel_size.is_some() {
            self.voxel_size = other.voxel_size;
        }
        if other.linear_choice.is_some() {
            self.linear_choice = other.linear_choice;
        }
        if other.max_plans_per_face.is_some() {
            self.max_plans_per_face = other.max_plans_per_face;
        }
    }
}
