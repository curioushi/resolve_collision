use ncollide3d::na;
use ncollide3d::shape::Cuboid;
use std::collections::{HashMap, VecDeque};

type Vertices = Vec<(f32, f32, f32)>;
type Indices = Vec<(u32, u32, u32)>;

pub fn to_rerun_mesh3d(vertices: &Vertices, indices: &Indices) -> rerun::Mesh3D {
    let mut flatten_vertices = Vec::new();
    let mut flatten_normals = Vec::new();
    let mut flatten_indices = Vec::new();
    let mut counter: u32 = 0;
    for (i, j, k) in indices.iter() {
        let (vi, vj, vk) = (
            vertices[*i as usize],
            vertices[*j as usize],
            vertices[*k as usize],
        );
        let normal = na::Vector3::new(vj.0 - vi.0, vj.1 - vi.1, vj.2 - vi.2)
            .cross(&na::Vector3::new(vk.0 - vi.0, vk.1 - vi.1, vk.2 - vi.2));
        flatten_vertices.push(vi);
        flatten_vertices.push(vj);
        flatten_vertices.push(vk);
        flatten_normals.push((normal.x, normal.y, normal.z));
        flatten_normals.push((normal.x, normal.y, normal.z));
        flatten_normals.push((normal.x, normal.y, normal.z));
        flatten_indices.push((counter, counter + 1, counter + 2));
        counter += 3;
    }
    rerun::Mesh3D::new(flatten_vertices)
        .with_triangle_indices(flatten_indices)
        .with_vertex_normals(flatten_normals)
}

pub fn bake_cuboids_by_free_fall(
    half_sizes: &Vec<(f32, f32, f32)>,
    initial_poses: &Vec<na::Isometry3<f32>>,
) -> Vec<na::Isometry3<f32>> {
    assert_eq!(half_sizes.len(), initial_poses.len());
    use rapier3d::prelude::*;
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(1000.0, 1000.0, 1.0).build();
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(vector![0.0, 0.0, -1.0])
        .build();
    let body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, body_handle, &mut rigid_body_set);

    /* Create the cuboids  */
    let mut body_handles = Vec::new();
    for i in 0..half_sizes.len() {
        let half_size = half_sizes[i];
        let initial_pose = initial_poses[i];
        let translation = initial_pose.translation;
        let rotaiton = initial_pose.rotation.euler_angles();
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![translation.x, translation.y, translation.z])
            .rotation(vector![rotaiton.0, rotaiton.1, rotaiton.2])
            .build();
        let collider = ColliderBuilder::cuboid(half_size.0, half_size.1, half_size.2)
            .restitution(0.7)
            .build();
        let body_handle = rigid_body_set.insert(rigid_body);
        body_handles.push(body_handle);
        collider_set.insert_with_parent(collider, body_handle, &mut rigid_body_set);
    }

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, 0.0, -9.81];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    /* Bake the simulation. */
    for _ in 0..400 {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &physics_hooks,
            &event_handler,
        );
    }

    let final_poses = body_handles
        .iter()
        .map(|h| {
            let body = &rigid_body_set[*h];
            let translation = body.translation();
            let quat = body.rotation();
            na::Isometry3::from_parts(
                na::Translation3::new(translation.x, translation.y, translation.z),
                na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    quat.w, quat.i, quat.j, quat.k,
                )),
            )
        })
        .collect::<Vec<_>>();
    final_poses
}

pub struct SceneNode {
    pub geometry: Option<Box<dyn Geometry>>,
    pub transform: Option<na::Isometry3<f32>>,
    pub children: HashMap<String, SceneNode>,
}

impl SceneNode {
    pub fn new() -> Self {
        Self {
            geometry: None,
            transform: None,
            children: HashMap::new(),
        }
    }
}

pub struct SceneTree {
    pub root: SceneNode,
}

pub struct SceneTreeIterator<'a> {
    stack: VecDeque<(String, &'a SceneNode)>,
}

impl<'a> SceneTreeIterator<'a> {
    pub fn new(root: &'a SceneNode) -> Self {
        let mut stack = VecDeque::new();
        stack.push_back(("".to_string(), root));
        Self { stack }
    }
}

impl<'a> Iterator for SceneTreeIterator<'a> {
    type Item = (String, &'a SceneNode);

    fn next(&mut self) -> Option<Self::Item> {
        if let Some((path, node)) = self.stack.pop_front() {
            for (name, child) in node.children.iter() {
                self.stack.push_back((format!("{}/{}", path, name), child));
            }
            Some((path, node))
        } else {
            None
        }
    }
}

impl SceneTree {
    pub fn new() -> Self {
        Self {
            root: SceneNode::new(),
        }
    }

    pub fn clear(&mut self) {
        self.root = SceneNode::new();
    }

    pub fn set_geometry(&mut self, path: &str, geometry: Box<dyn Geometry>) {
        let mut node = &mut self.root;
        for name in path.split('/') {
            if name.is_empty() {
                continue;
            }
            if node.children.contains_key(name) {
                node = node.children.get_mut(name).unwrap();
            } else {
                let child = SceneNode::new();
                node.children.insert(name.to_string(), child);
                node = node.children.get_mut(name).unwrap();
            }
        }
        node.geometry = Some(geometry);
    }

    pub fn get_geometry(&self, path: &str) -> Option<&Box<dyn Geometry>> {
        let mut node = &self.root;
        for name in path.split('/') {
            if name.is_empty() {
                continue;
            }
            if node.children.contains_key(name) {
                node = node.children.get(name).unwrap();
            } else {
                return None;
            }
        }
        node.geometry.as_ref()
    }

    pub fn set_transform(&mut self, path: &str, transform: na::Isometry3<f32>) {
        let mut node = &mut self.root;
        for name in path.split('/') {
            if name.is_empty() {
                continue;
            }
            if node.children.contains_key(name) {
                node = node.children.get_mut(name).unwrap();
            } else {
                return;
            }
        }
        node.transform = Some(transform);
    }

    pub fn get_transform(&self, path: &str) -> Option<&na::Isometry3<f32>> {
        let mut node = &self.root;
        for name in path.split('/') {
            if name.is_empty() {
                continue;
            }
            if node.children.contains_key(name) {
                node = node.children.get(name).unwrap();
            } else {
                return None;
            }
        }
        node.transform.as_ref()
    }

    pub fn iter(&self) -> SceneTreeIterator {
        SceneTreeIterator::new(&self.root)
    }
}

pub enum GeometryType {
    Ground,
    Carton,
    Container,
}

pub trait Geometry: Send + Sync {
    fn type_id(&self) -> GeometryType;
    fn size(&self) -> (f32, f32, f32);
    fn vertices(&self) -> Vertices;
    fn indices(&self) -> Indices;
}

pub struct Carton {
    cuboid: Cuboid<f32>,
}

impl Carton {
    pub fn new(size_x: f32, size_y: f32, size_z: f32) -> Self {
        Self {
            cuboid: Cuboid::new(na::Vector3::new(
                size_x * 0.5f32,
                size_y * 0.5f32,
                size_z * 0.5f32,
            )),
        }
    }
}

impl Geometry for Carton {
    fn size(&self) -> (f32, f32, f32) {
        let half_extents = self.cuboid.half_extents;
        (
            half_extents.x * 2.0,
            half_extents.y * 2.0,
            half_extents.z * 2.0,
        )
    }
    fn type_id(&self) -> GeometryType {
        GeometryType::Carton
    }
    fn vertices(&self) -> Vertices {
        let half_extents = self.cuboid.half_extents;
        vec![
            (-half_extents.x, -half_extents.y, -half_extents.z),
            (half_extents.x, -half_extents.y, -half_extents.z),
            (half_extents.x, half_extents.y, -half_extents.z),
            (-half_extents.x, half_extents.y, -half_extents.z),
            (-half_extents.x, -half_extents.y, half_extents.z),
            (half_extents.x, -half_extents.y, half_extents.z),
            (half_extents.x, half_extents.y, half_extents.z),
            (-half_extents.x, half_extents.y, half_extents.z),
        ]
    }

    fn indices(&self) -> Indices {
        vec![
            (0, 2, 1),
            (0, 3, 2),
            (4, 5, 6),
            (4, 6, 7),
            (0, 1, 5),
            (0, 5, 4),
            (3, 6, 2),
            (3, 7, 6),
            (1, 2, 6),
            (1, 6, 5),
            (0, 7, 3),
            (0, 4, 7),
        ]
    }
}

pub struct Container {
    size: (f32, f32, f32),
    vertices: Vertices,
    indices: Indices,
}

impl Container {
    pub fn new(size_x: f32, size_y: f32, size_z: f32) -> Self {
        let hsize_y = size_y * 0.5f32;
        let vertices = vec![
            (0.0, hsize_y, 0.0),
            (0.0, -hsize_y, 0.0),
            (size_x, -hsize_y, 0.0),
            (size_x, hsize_y, 0.0),
            (0.0, hsize_y, size_z),
            (0.0, -hsize_y, size_z),
            (size_x, -hsize_y, size_z),
            (size_x, hsize_y, size_z),
        ];
        let indices = vec![
            (0, 2, 1),
            (0, 3, 2),
            (4, 5, 6),
            (4, 6, 7),
            (3, 6, 2),
            (3, 7, 6),
            (1, 2, 6),
            (1, 6, 5),
            (0, 7, 3),
            (0, 4, 7),
        ];
        Self {
            size: (size_x, size_y, size_z),
            vertices,
            indices,
        }
    }
}

impl Geometry for Container {
    fn size(&self) -> (f32, f32, f32) {
        self.size
    }
    fn type_id(&self) -> GeometryType {
        GeometryType::Container
    }
    fn vertices(&self) -> Vertices {
        self.vertices.clone()
    }
    fn indices(&self) -> Indices {
        self.indices.clone()
    }
}

pub struct Ground {
    size: f32,
}

impl Ground {
    pub fn new(size: f32) -> Self {
        Self { size }
    }
}

impl Geometry for Ground {
    fn size(&self) -> (f32, f32, f32) {
        (self.size, self.size, 0.0)
    }
    fn type_id(&self) -> GeometryType {
        GeometryType::Ground
    }
    fn vertices(&self) -> Vertices {
        vec![
            (-self.size, -self.size, 0.0),
            (self.size, -self.size, 0.0),
            (self.size, self.size, 0.0),
            (-self.size, self.size, 0.0),
        ]
    }
    fn indices(&self) -> Indices {
        vec![(0, 1, 2), (0, 2, 3)]
    }
}
