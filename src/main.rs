use ncollide3d as n3d;
use ncollide3d::na;
use ncollide3d::shape::Cuboid;
use serde::{Deserialize, Serialize};

fn shrink_by_point<T>(cuboid: &Cuboid<T>, point: &na::Point3<T>) -> Cuboid<T>
where
    T: Copy + na::RealField,
{
    let half_extents = cuboid.half_extents;
    let shrink_length = half_extents - point.coords.map(|x| x.abs());
    let shrink_length = shrink_length.map(|x| x.max(T::zero()));
    let min_index = shrink_length.imin();
    let mut new_half_extents = half_extents;
    new_half_extents[min_index] -= shrink_length[min_index] + T::from_f32(1e-6).unwrap();
    Cuboid::new(new_half_extents)
}

fn resolve_contact_by_shrink(
    m1: &na::Isometry3<f64>,
    c1: &mut Cuboid<f64>,
    m2: &na::Isometry3<f64>,
    c2: &mut Cuboid<f64>,
) {
    let mut counter = 0;
    while let Some(contact) = n3d::query::contact(m1, c1, m2, c2, 0.0) {
        if counter > 10 {
            break;
        }
        let middle_point = contact.world2 + contact.normal.into_inner() * contact.depth * 0.5;
        c1.half_extents =
            shrink_by_point(c1, &m1.inverse_transform_point(&middle_point)).half_extents;
        c2.half_extents =
            shrink_by_point(c2, &m2.inverse_transform_point(&middle_point)).half_extents;
        println!("depth: {}", contact.depth);
        counter += 1;
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct CubeSerde {
    tf: Vec<Vec<f64>>,
    size: Vec<f64>,
}

struct CuboidWithTf {
    cuboid: Cuboid<f64>,
    tf: na::Isometry3<f64>,
}

fn main() {
    let json_filepath = "/home/shq/Projects/mycode/resolve_collision/stacking.json";
    let mut cubes: Vec<CuboidWithTf> = Vec::new();
    if let Ok(json_data) = std::fs::read_to_string(json_filepath) {
        let cubes_serde: Vec<CubeSerde> = serde_json::from_str(&json_data).unwrap();
        for cude_serde in cubes_serde.iter() {
            let rotmat = na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[
                na::Vector3::new(
                    cude_serde.tf[0][0],
                    cude_serde.tf[0][1],
                    cude_serde.tf[0][2],
                ),
                na::Vector3::new(
                    cude_serde.tf[1][0],
                    cude_serde.tf[1][1],
                    cude_serde.tf[1][2],
                ),
                na::Vector3::new(
                    cude_serde.tf[2][0],
                    cude_serde.tf[2][1],
                    cude_serde.tf[2][2],
                ),
            ]));
            let tf = na::Isometry3::from_parts(
                na::Translation3::new(
                    cude_serde.tf[0][3],
                    cude_serde.tf[1][3],
                    cude_serde.tf[2][3],
                ),
                na::UnitQuaternion::from_rotation_matrix(&rotmat),
            );
            let size = na::Vector3::new(cude_serde.size[0], cude_serde.size[1], cude_serde.size[2]);
            let cuboid = Cuboid::new(size / 2.0);
            cubes.push(CuboidWithTf { cuboid, tf });
        }
    }

    for i in 0..cubes.len() {
        for j in i + 1..cubes.len() {
            let mut cuboid1 = cubes[i].cuboid;
            let tf1 = cubes[i].tf;
            let mut cuboid2 = cubes[j].cuboid;
            let tf2 = cubes[j].tf;
            println!("==========================================");
            println!("solve contact between cube{} and cube{}", i, j);
            println!("cube{}:", i);
            println!("- half_extents: {:?}", cuboid1.half_extents);
            println!("- tf: {:?}", tf1);
            println!("cube{}:", j);
            println!("- half_extents: {:?}", cuboid2.half_extents);
            println!("- tf: {:?}", tf2);
            resolve_contact_by_shrink(&tf1, &mut cuboid1, &tf2, &mut cuboid2);
            break;
        }
    }

    // let mut cube1 = Cuboid::new(na::Vector3::new(1.0, 1.0, 1.0));
    // let m1 = na::Isometry3::new(na::Vector3::new(1.5, 0.0, 0.0), na::zero());
    // let mut cube2 = Cuboid::new(na::Vector3::new(1.0, 1.0, 1.0));
    // let m2 = na::Isometry3::new(na::zero(), na::zero());
    // resolve_contact_by_shrink(&m1, &mut cube1, &m2, &mut cube2);
    // println!("cube1: {:?}", cube1.half_extents);
    // println!("cube2: {:?}", cube2.half_extents);
}
