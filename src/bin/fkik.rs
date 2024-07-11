use ncollide3d::na;
use rand::Rng;
use resolve_collision::fanuc::{fk, ik};
use std::f64::consts::PI;

fn main() {
    let mut poses = Vec::new();
    let mut rng = rand::thread_rng();
    let mut sum = std::time::Duration::new(0, 0);
    for _ in 0..1000000 {
        let a1 = rng.gen_range(-PI..PI);
        let a2 = rng.gen_range(-PI..PI);
        let a3 = rng.gen_range(-PI..PI);
        let a4 = rng.gen_range(-PI..PI);
        let a5 = rng.gen_range(-PI..PI);
        let a6 = rng.gen_range(-PI..PI);
        let t1 = std::time::Instant::now();
        let end_pose = fk(a1, a2, a3, a4, a5, a6);
        let t2 = std::time::Instant::now();
        let duration = t2 - t1;
        sum += duration;
        poses.push(end_pose);
    }
    println!("FK average cost: {:?}", sum / 1000000);

    let mut all_solutions = Vec::new();
    let mut sum = std::time::Duration::new(0, 0);
    for _ in 0..1000000 {
        let target = (
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-PI..PI),
            rng.gen_range(-PI..PI),
            rng.gen_range(-PI..PI),
        );
        let t1 = std::time::Instant::now();
        let solutions = ik(&na::Isometry3::from_parts(
            na::Translation3::new(target.0, target.1, target.2),
            na::UnitQuaternion::from_euler_angles(target.3, target.4, target.5),
        ));
        let t2 = std::time::Instant::now();
        let duration = t2 - t1;
        sum += duration;
        all_solutions.push(solutions);
    }
    println!("IK average cost: {:?}", sum / 1000000);

    let target = na::Isometry3::from_parts(
        na::Translation3::new(0.726, -0.765, 0.875),
        na::UnitQuaternion::from_quaternion(na::Quaternion::new(0.0, 0.7071068, 0.0, 0.7071068)),
    );
    let solutions = ik(&target);

    println!("--------------------------------------------------");
    println!("Target: {:?}, {:?}", target.translation, target.rotation);

    for (a1, a2, a3, a4, a5, a6) in solutions.iter() {
        println!(
            "IK = {}, {}, {}, {}, {}, {}",
            a1.to_degrees(),
            a2.to_degrees(),
            a3.to_degrees() - a2.to_degrees(),
            a4.to_degrees(),
            a5.to_degrees(),
            a6.to_degrees()
        );
    }

    for solution in solutions.iter() {
        let pose = fk(
            solution.0, solution.1, solution.2, solution.3, solution.4, solution.5,
        );
        println!("FK result: {:?}, {:?}", pose.translation, pose.rotation);
    }
}
