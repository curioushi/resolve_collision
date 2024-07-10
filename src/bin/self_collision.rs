use ncollide3d::na;
use ncollide3d::query;
use ncollide3d::shape::{ConvexHull, TriMesh};
use stl_io::read_stl;

struct RobotArm {
    links: Vec<TriMesh<f64>>,
    links_convex: Vec<ConvexHull<f64>>,
}

impl RobotArm {
    fn new(links: Vec<TriMesh<f64>>) -> Self {
        let links_convex = links
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

fn main() {
    let robot_arm = RobotArm::new(vec![
        read_stl_file("data/fanuc/convex/link0.stl"),
        read_stl_file("data/fanuc/convex/link1.stl"),
        read_stl_file("data/fanuc/convex/link2.stl"),
        read_stl_file("data/fanuc/convex/link3.stl"),
        read_stl_file("data/fanuc/convex/link4.stl"),
        read_stl_file("data/fanuc/convex/link5.stl"),
        read_stl_file("data/fanuc/convex/link6.stl"),
    ]);

    for i in 0..robot_arm.links.len() {
        let link = &robot_arm.links[i];
        let link_convex = &robot_arm.links_convex[i];
        println!("Link {}: {} vertices", i, link.points().len());
        println!("Link Convex {}: {} vertices", i, link_convex.points().len());
    }

    let t1 = std::time::Instant::now();
    let mut count = 0;
    for _ in 0..1000 {
        for i in 0..robot_arm.links.len() - 1 {
            let link1 = &robot_arm.links_convex[i];
            let link2 = &robot_arm.links_convex[i + 1];
            let prox = query::proximity(
                &na::Isometry3::identity(),
                link1,
                &na::Isometry3::identity(),
                link2,
                0.0,
            );
            // println!("{} -> {}: {:?}", i, i + 1, prox);
            count += 1;
        }
    }
    let t2 = std::time::Instant::now();
    println!("Avg Time: {:?}", (t2 - t1) / count);
}
