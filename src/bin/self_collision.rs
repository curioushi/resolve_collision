use ncollide3d::na;
use ncollide3d::query;
use ncollide3d::shape::{ConvexHull, TriMesh};
use rand::Rng;
use resolve_collision::robot::{fk0, fk01, fk12, fk23, fk34, fk45, fk56};
use stl_io::read_stl;

struct RobotArm {
    links: Vec<TriMesh<f64>>,
    links_convex: Vec<ConvexHull<f64>>,
}

impl RobotArm {
    fn new(links: Vec<TriMesh<f64>>, links_convex: Vec<TriMesh<f64>>) -> Self {
        let links_convex = links_convex
            .iter()
            .map(|link| ConvexHull::try_from_points(link.points()).unwrap())
            .collect::<Vec<_>>();
        RobotArm {
            links,
            links_convex,
        }
    }
}

fn read_stl_file(file_path: &str) -> TriMesh<f64> {
    let file = std::fs::File::open(file_path).unwrap();
    let mut reader = std::io::BufReader::new(file);
    let stl = read_stl(&mut reader).expect("Failed to read STL file");
    let vertices = stl
        .vertices
        .iter()
        .map(|v| na::Point3::new(v[0] as f64, v[1] as f64, v[2] as f64))
        .collect();
    let indices = stl
        .faces
        .iter()
        .map(|f| na::Point3::new(f.vertices[0], f.vertices[1], f.vertices[2]))
        .collect();

    let mesh = TriMesh::new(vertices, indices, None);
    mesh
}

fn convert_trimesh_to_rerun(
    mesh: &TriMesh<f64>,
    color: (u8, u8, u8),
) -> (
    Vec<(f32, f32, f32)>,
    Vec<(u32, u32, u32)>,
    Vec<(f32, f32, f32)>,
    Vec<(u8, u8, u8)>,
) {
    let vertices = mesh
        .points()
        .iter()
        .map(|p| (p.coords[0] as f32, p.coords[1] as f32, p.coords[2] as f32))
        .collect::<Vec<_>>();
    let indices = mesh
        .faces()
        .iter()
        .map(|f| {
            (
                f.indices[0] as u32,
                f.indices[1] as u32,
                f.indices[2] as u32,
            )
        })
        .collect::<Vec<_>>();
    let normals = indices
        .iter()
        .map(|(i0, i1, i2)| {
            let p0 = mesh.points()[*i0 as usize];
            let p1 = mesh.points()[*i1 as usize];
            let p2 = mesh.points()[*i2 as usize];
            let n = na::Vector3::cross(&(p1 - p0), &(p2 - p0)).normalize();
            (n.x as f32, n.y as f32, n.z as f32)
        })
        .collect::<Vec<_>>();
    let mut vertex_normals = vertices
        .iter()
        .map(|_| na::Vector3::new(0.0f32, 0.0f32, 0.0f32))
        .collect::<Vec<_>>();
    for (i, (i0, i1, i2)) in indices.iter().enumerate() {
        let n = na::Vector3::new(normals[i].0, normals[i].1, normals[i].2);
        vertex_normals[*i0 as usize] += n;
        vertex_normals[*i1 as usize] += n;
        vertex_normals[*i2 as usize] += n;
    }
    let vertex_normals = vertex_normals
        .iter()
        .map(|n| {
            let n = n.normalize();
            (n.x as f32, n.y as f32, n.z as f32)
        })
        .collect::<Vec<_>>();
    let vertex_colors = vertices.iter().map(|_| color).collect::<Vec<_>>();
    (vertices, indices, vertex_normals, vertex_colors)
}

const FANUC_YELLOW: (u8, u8, u8) = (247, 202, 1);
const WARNING_RED: (u8, u8, u8) = (255, 0, 0);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot_arm = RobotArm::new(
        vec![
            read_stl_file("data/fanuc/mesh/link0.stl"),
            read_stl_file("data/fanuc/mesh/link1.stl"),
            read_stl_file("data/fanuc/mesh/link2.stl"),
            read_stl_file("data/fanuc/mesh/link3.stl"),
            read_stl_file("data/fanuc/mesh/link4.stl"),
            read_stl_file("data/fanuc/mesh/link5.stl"),
            read_stl_file("data/fanuc/mesh/link6.stl"),
        ],
        vec![
            read_stl_file("data/fanuc/convex/link0.stl"),
            read_stl_file("data/fanuc/convex/link1.stl"),
            read_stl_file("data/fanuc/convex/link2.stl"),
            read_stl_file("data/fanuc/convex/link3.stl"),
            read_stl_file("data/fanuc/convex/link4.stl"),
            read_stl_file("data/fanuc/convex/link5.stl"),
            read_stl_file("data/fanuc/convex/link6.stl"),
        ],
    );

    let rec = rerun::RecordingStreamBuilder::new("self_collision").spawn()?;
    let mut frame = 0;
    let mut rng = rand::thread_rng();
    let origin_0 = fk0();
    let origin_1 = origin_0 * fk01(0.0);
    let origin_2 = origin_1 * fk12(0.0);
    let origin_3 = origin_2 * fk23(0.0);
    let origin_4 = origin_3 * fk34(0.0);
    let origin_5 = origin_4 * fk45(0.0);
    let origin_6 = origin_5 * fk56(0.0);
    let mut avg_collision_matrix = vec![vec![0.0; robot_arm.links.len()]; robot_arm.links.len()];
    loop {
        frame += 1;
        rec.set_time_sequence("frame", frame);
        let a1 = (rng.gen_range(-185..185) as f64).to_radians();
        let a2 = (rng.gen_range(-85..130) as f64).to_radians();
        let a3 = (rng.gen_range(-120..80) as f64).to_radians() + a2;
        let a4 = (rng.gen_range(-200..200) as f64).to_radians();
        let a5 = (rng.gen_range(-140..140) as f64).to_radians();
        let a6 = (rng.gen_range(-270..270) as f64).to_radians();
        let tf0 = fk0();
        let tf1 = tf0 * fk01(a1);
        let tf2 = tf1 * fk12(a2);
        let tf3 = tf2 * fk23(a3);
        let tf4 = tf3 * fk34(a4);
        let tf5 = tf4 * fk45(a5);
        let tf6 = tf5 * fk56(a6);
        let tf0 = tf0 * origin_0.inverse();
        let tf1 = tf1 * origin_1.inverse();
        let tf2 = tf2 * origin_2.inverse();
        let tf3 = tf3 * origin_3.inverse();
        let tf4 = tf4 * origin_4.inverse();
        let tf5 = tf5 * origin_5.inverse();
        let tf6 = tf6 * origin_6.inverse();
        let tfs = vec![tf0, tf1, tf2, tf3, tf4, tf5, tf6];

        let t1 = std::time::Instant::now();
        let mut collision_matrix = vec![vec![false; robot_arm.links.len()]; robot_arm.links.len()];
        for i in [0, 1] {
            let link1 = &robot_arm.links_convex[i];
            let tf1 = tfs[i];
            for j in [4, 5, 6] {
                let link2 = &robot_arm.links_convex[j];
                let tf2 = tfs[j];
                let prox = query::proximity(&tf1, link1, &tf2, link2, 0.0);
                match prox {
                    query::Proximity::Intersecting => {
                        collision_matrix[i][j] = true;
                        collision_matrix[j][i] = true;
                        avg_collision_matrix[i][j] += 1.0;
                        avg_collision_matrix[j][i] += 1.0;
                    }
                    _ => {}
                }
            }
        }
        let t2 = std::time::Instant::now();

        for (i, link) in robot_arm.links.iter().enumerate() {
            let is_collision = collision_matrix[i].iter().any(|&x| x);
            let color = if is_collision {
                WARNING_RED
            } else {
                FANUC_YELLOW
            };
            let (vertices, indices, vertex_normals, vertex_colors) =
                convert_trimesh_to_rerun(link, color);
            rec.set_time_sequence("frame", frame);
            rec.log(
                format!("link_{}", i),
                &rerun::Mesh3D::new(vertices)
                    .with_triangle_indices(indices)
                    .with_vertex_normals(vertex_normals)
                    .with_vertex_colors(vertex_colors),
            )?;
        }

        let mut collision_matrix_str = String::new();
        collision_matrix_str.push_str("# Collision Matrix\n");
        for i in 0..robot_arm.links.len() {
            for j in 0..robot_arm.links.len() {
                if collision_matrix[i][j] {
                    collision_matrix_str.push_str("X");
                } else {
                    collision_matrix_str.push_str("0");
                }
            }
            collision_matrix_str.push_str("\n");
        }
        rec.log(
            "collision_matrix",
            &rerun::TextDocument::new(collision_matrix_str),
        )?;
        println!("Self Collision Time: {:?}", (t2 - t1));

        for (i, tf) in tfs.iter().enumerate() {
            rec.log(
                format!("link_{}", i),
                &rerun::Transform3D::from_translation_rotation(
                    (
                        tf.translation.x as f32,
                        tf.translation.y as f32,
                        tf.translation.z as f32,
                    ),
                    rerun::Quaternion::from_xyzw([
                        tf.rotation.i as f32,
                        tf.rotation.j as f32,
                        tf.rotation.k as f32,
                        tf.rotation.w as f32,
                    ]),
                ),
            )?;
        }
        if frame == 3000 {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }

    for i in 0..robot_arm.links.len() {
        for j in 0..robot_arm.links.len() {
            avg_collision_matrix[i][j] /= 3000.0;
        }
        println!("{:?}", avg_collision_matrix[i]);
    }
    Ok(())
}
