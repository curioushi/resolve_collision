use ncollide3d::{na, simba::scalar::SubsetOf};
use rand::Rng;

#[rustfmt::skip]
fn fk(a1: f64, a2: f64, a3: f64, a4: f64, a5: f64, a6: f64) -> na::Isometry3<f64> {
    let (c1, c2, c3, c4, c5, c6) = (a1.cos(), a2.cos(), a3.cos(), a4.cos(), a5.cos(), a6.cos());
    let (s1, s2, s3, s4, s5, s6) = (a1.sin(), a2.sin(), a3.sin(), a4.sin(), a5.sin(), a6.sin());
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        c1*c3*c4*c5*c6*s2-c1*c2*c4*c5*c6*s3-c5*c6*s1*s4-c1*c2*c3*c6*s5-c1*c6*s2*s3*s5-c4*s1*s6-c1*c3*s2*s4*s6+c1*c2*s3*s4*s6,
        c4*c6*s1+c1*c3*c6*s2*s4-c1*c2*c6*s3*s4+c1*c3*c4*c5*s2*s6-c1*c2*c4*c5*s3*s6-c5*s1*s4*s6-c1*c2*c3*s5*s6-c1*s2*s3*s5*s6,
        c1*c2*c3*c5+c1*c5*s2*s3+c1*c3*c4*s2*s5-c1*c2*c4*s3*s5-s1*s4*s5,
        0.075*c1+0.89*c1*c2*c3+0.09*c1*c2*c3*c5+0.84*c1*s2+0.215*c1*c3*s2-0.215*c1*c2*s3+0.89*c1*s2*s3+0.09*c1*c5*s2*s3+0.09*c1*c3*c4*s2*s5-0.09*c1*c2*c4*s3*s5-0.09*s1*s4*s5,
        c3*c4*c5*c6*s1*s2-c2*c4*c5*c6*s1*s3+c1*c5*c6*s4-c2*c3*c6*s1*s5-c6*s1*s2*s3*s5+c1*c4*s6-c3*s1*s2*s4*s6+c2*s1*s3*s4*s6,
        -c1*c4*c6+c3*c6*s1*s2*s4-c2*c6*s1*s3*s4+c3*c4*c5*s1*s2*s6-c2*c4*c5*s1*s3*s6+c1*c5*s4*s6-c2*c3*s1*s5*s6-s1*s2*s3*s5*s6,
        c2*c3*c5*s1+c5*s1*s2*s3+c3*c4*s1*s2*s5-c2*c4*s1*s3*s5+c1*s4*s5,
        0.075*s1+0.89*c2*c3*s1+0.09*c2*c3*c5*s1+0.84*s1*s2+0.215*c3*s1*s2-0.215*c2*s1*s3+0.89*s1*s2*s3+0.09*c5*s1*s2*s3+0.09*c3*c4*s1*s2*s5-0.09*c2*c4*s1*s3*s5+0.09*c1*s4*s5,
        c2*c3*c4*c5*c6+c4*c5*c6*s2*s3+c3*c6*s2*s5-c2*c6*s3*s5-c2*c3*s4*s6-s2*s3*s4*s6,
        c2*c3*c6*s4+c6*s2*s3*s4+c2*c3*c4*c5*s6+c4*c5*s2*s3*s6+c3*s2*s5*s6-c2*s3*s5*s6,
        -c3*c5*s2+c2*c5*s3+c2*c3*c4*s5+c4*s2*s3*s5,
        0.84*c2+0.215*c2*c3-0.89*c3*s2-0.09*c3*c5*s2+0.89*c2*s3+0.09*c2*c5*s3+0.215*s2*s3+0.09*c2*c3*c4*s5+0.09*c4*s2*s3*s5 ,
        0.0, 0.0, 0.0, 1.0
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

fn main() {
    let mut poses = Vec::new();
    let mut rng = rand::thread_rng();
    let mut sum = std::time::Duration::new(0, 0);
    for _ in 0..1000000 {
        let a1 = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);
        let a2 = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);
        let a3 = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);
        let a4 = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);
        let a5 = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);
        let a6 = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);
        let t1 = std::time::Instant::now();
        let end_pose = fk(a1, a2, a3, a4, a5, a6);
        let t2 = std::time::Instant::now();
        let duration = t2 - t1;
        sum += duration;
        poses.push(end_pose);
    }
    println!("Poses: {:?}", poses.len());
    println!("Average time: {:?}", sum / 1000000);

    println!(
        "FK(45,45,65,45,45,45) = {:?}",
        fk(
            45.0f64.to_radians(),
            45.0f64.to_radians(),
            65.0f64.to_radians(),
            45.0f64.to_radians(),
            45.0f64.to_radians(),
            45.0f64.to_radians()
        )
    );
}
