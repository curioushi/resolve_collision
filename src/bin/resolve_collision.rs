use clap::Parser;
use ncollide3d as n3d;
use ncollide3d::na;
use ncollide3d::shape::Cuboid;
use resolve_collision::common::{CubeSerde, CuboidWithTf};

fn shrink_by_point<T>(
    tf: &na::Isometry3<T>,
    cuboid: &Cuboid<T>,
    point: &na::Point3<T>,
    normal: &na::UnitVector3<T>,
) -> Cuboid<T>
where
    T: Copy + na::RealField,
{
    let half_extents = cuboid.half_extents;
    let shrink_length = half_extents - tf.inverse_transform_point(point).coords.map(|x| x.abs());
    let shrink_length = shrink_length.map(|x| x.max(T::zero()));
    let index = tf
        .inverse_transform_unit_vector(normal)
        .into_inner()
        .map(|x| x.abs())
        .imax();
    let mut new_half_extents = half_extents;
    new_half_extents[index] -= shrink_length[index] + T::from_f32(1e-6).unwrap();
    Cuboid::new(new_half_extents)
}

fn resolve_contact_by_shrink(
    m1: &na::Isometry3<f64>,
    c1: &mut Cuboid<f64>,
    m2: &na::Isometry3<f64>,
    c2: &mut Cuboid<f64>,
    only_shrink_first: bool,
) -> u32 {
    let mut counter: u32 = 0;
    while let Some(contact) = n3d::query::contact(m1, c1, m2, c2, 0.0) {
        if counter > 10 {
            break;
        }
        let normal = contact.normal;
        if only_shrink_first {
            let point = contact.world2;
            c1.half_extents = shrink_by_point(&m1, c1, &point, &normal).half_extents;
        } else {
            let point = contact.world2 + contact.normal.into_inner() * contact.depth * 0.5;
            c1.half_extents = shrink_by_point(&m1, c1, &point, &normal).half_extents;
            c2.half_extents = shrink_by_point(&m2, c2, &point, &normal).half_extents;
        }
        counter += 1;
    }
    counter
}

#[derive(Parser, Debug)]
struct Cli {
    #[arg(short, long)]
    input: String,

    #[arg(short, long)]
    output: String,

    #[arg(long, value_delimiter = ',')]
    min_bound: Option<Vec<f64>>,

    #[arg(long, value_delimiter = ',')]
    max_bound: Option<Vec<f64>>,
}

fn main() {
    let args = Cli::parse();
    match (args.min_bound.clone(), args.max_bound.clone()) {
        (Some(min_bound), Some(max_bound)) => {
            // check if min_bound and max_bound have the same length
            if min_bound.len() != 3 || max_bound.len() != 3 {
                panic!("min_bound and max_bound must have length 3");
            }

            // check if min_bound is less than max_bound
            for i in 0..3 {
                if min_bound[i] > max_bound[i] {
                    panic!("min_bound must be less than max_bound");
                }
            }
        }
        (None, None) => {}
        _ => {
            panic!("min_bound and max_bound must be both specified or both not specified");
        }
    }

    let start_time = std::time::Instant::now();
    let mut cubes: Vec<CuboidWithTf> = Vec::new();
    if let Ok(json_data) = std::fs::read_to_string(args.input) {
        let cubes_serde: Vec<CubeSerde> = serde_json::from_str(&json_data).unwrap();
        for c in cubes_serde.iter() {
            cubes.push(CuboidWithTf::from_cube_serde(c));
        }
    }

    for i in 0..cubes.len() {
        for j in i + 1..cubes.len() {
            let mut cuboid1 = cubes[i].cuboid;
            let tf1 = cubes[i].tf;
            let mut cuboid2 = cubes[j].cuboid;
            let tf2 = cubes[j].tf;
            resolve_contact_by_shrink(&tf1, &mut cuboid1, &tf2, &mut cuboid2, true);
            cubes[i].cuboid = cuboid1;
            cubes[j].cuboid = cuboid2;
        }
    }

    if let (Some(min_bound), Some(max_bound)) = (args.min_bound, args.max_bound) {
        let size = na::Vector3::new(
            max_bound[0] - min_bound[0],
            max_bound[1] - min_bound[1],
            max_bound[2] - min_bound[2],
        );
        let center = na::Point3::new(
            (min_bound[0] + max_bound[0]) / 2.0,
            (min_bound[1] + max_bound[1]) / 2.0,
            (min_bound[2] + max_bound[2]) / 2.0,
        );
        let translations: Vec<na::Translation3<f64>> = vec![
            na::Translation3::new(center.x - size[0], center.y, center.z),
            na::Translation3::new(center.x + size[0], center.y, center.z),
            na::Translation3::new(center.x, center.y - size[1], center.z),
            na::Translation3::new(center.x, center.y + size[1], center.z),
            na::Translation3::new(center.x, center.y, center.z - size[2]),
            na::Translation3::new(center.x, center.y, center.z + size[2]),
        ];
        let walls: Vec<CuboidWithTf> = translations
            .iter()
            .map(|t| CuboidWithTf {
                cuboid: Cuboid::new(size / 2.0),
                tf: na::Isometry3::from_parts(*t, na::UnitQuaternion::identity()),
            })
            .collect();

        for i in 0..cubes.len() {
            for wall in walls.iter() {
                let mut cuboid1 = cubes[i].cuboid;
                let tf1 = cubes[i].tf;
                let mut cuboid2 = wall.cuboid;
                let tf2 = wall.tf;
                resolve_contact_by_shrink(&tf1, &mut cuboid1, &tf2, &mut cuboid2, false);
                cubes[i].cuboid = cuboid1;
            }
        }
    }

    let cube_serdes = cubes
        .iter()
        .map(|c| {
            let tf = c.tf.to_homogeneous();
            let size = c.cuboid.half_extents * 2.0;
            CubeSerde {
                tf: tf
                    .row_iter()
                    .map(|row| row.iter().map(|x| *x).collect())
                    .collect(),
                size: size.iter().map(|x| *x).collect(),
            }
        })
        .collect::<Vec<CubeSerde>>();

    let json_data = serde_json::to_string(&cube_serdes).unwrap();
    std::fs::write(args.output, json_data).unwrap();
    let end_time = std::time::Instant::now();
    println!("Elapsed time: {:?}", end_time - start_time);
}
