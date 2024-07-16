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

pub trait Geometry: Send + Sync {
    fn size(&self) -> (f32, f32, f32);
    fn vertices(&self) -> Vertices;
    fn indices(&self) -> Indices;
}

pub struct Carton {
    cuboid: Cuboid<f32>,
}

impl Carton {
    pub fn new(half_size_x: f32, half_size_y: f32, half_size_z: f32) -> Self {
        Self {
            cuboid: Cuboid::new(na::Vector3::new(half_size_x, half_size_y, half_size_z)),
        }
    }
}

impl Geometry for Carton {
    fn size(&self) -> (f32, f32, f32) {
        let half_extents = self.cuboid.half_extents;
        (
            half_extents.x * 2.0f32,
            half_extents.y * 2.0f32,
            half_extents.z * 2.0f32,
        )
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
