use ncollide3d::{na, simba::scalar::SubsetOf};
use std::f64::consts::{PI, TAU};

#[rustfmt::skip]
pub fn fk(a1: f64, a2: f64, a3: f64, a4: f64, a5: f64, a6: f64) -> na::Isometry3<f64> {
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

pub fn fk03(a1: f64, a2: f64, a3: f64) -> na::Isometry3<f64> {
    let (c1, c2, c3) = (a1.cos(), a2.cos(), a3.cos());
    let (s1, s2, s3) = (a1.sin(), a2.sin(), a3.sin());
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        c1 * c3 * s2 - c1 * c2 * s3,
        -c1 * c2 * c3 - c1 * s2 * s3,
        s1,
        0.075 * c1 + 0.84 * c1 * s2,
        c3 * s1 * s2 - c2 * s1 * s3,
        -c2 * c3 * s1 - s1 * s2 * s3,
        -c1,
        0.075 * s1 + 0.84 * s1 * s2,
        c2 * c3 + s2 * s3,
        c3 * s2 - c2 * s3,
        0.,
        0.84 * c2,
        0.0,
        0.0,
        0.0,
        1.0,
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

pub fn solve_wrist_angles(r: &na::Rotation3<f64>) -> ((f64, f64, f64), (f64, f64, f64)) {
    if r[(1, 2)] == 1.0 {
        // singular point
        let r00 = r[(0, 0)];
        let r01 = r[(0, 1)];
        let a4a6 = (-r01).atan2(r00);
        ((0.0, 0.0, a4a6), (PI, 0.0, a4a6 - PI))
    } else {
        let r10 = r[(1, 0)];
        let r12 = r[(1, 2)];
        let r11 = r[(1, 1)];
        let r02 = r[(0, 2)];
        let r22 = r[(2, 2)];
        let a5 = (r10 * r10 + r11 * r11).sqrt().atan2(r12);
        let a4 = r22.atan2(-r02);
        let a6 = (-r11).atan2(r10);
        ((a4, a5, a6), (a4 + PI, -a5, a6 + PI))
    }
}

pub fn ik(tf_0_flange: &na::Isometry3<f64>) -> Vec<(f64, f64, f64, f64, f64, f64)> {
    const K1: f64 = 0.075;
    const L1: f64 = 0.84;
    const SQUARE_L1: f64 = L1 * L1;
    const L2: f64 = 0.9156008955871549;
    const SQUARE_L2: f64 = L2 * L2;
    const GAMMA1: f64 = 1.8078281373462155;
    let tf_flange_6 = na::Isometry3::from_parts(
        na::Translation3::new(0.0, 0.0, -0.090),
        na::UnitQuaternion::from_quaternion(na::Quaternion::new(0.0, 1.0, 0.0, 0.0)),
    );
    let tf_0_6 = tf_0_flange * tf_flange_6;
    let wrist_position = tf_0_6.translation;
    let x: f64 = wrist_position.x;
    let y: f64 = wrist_position.y;
    let z: f64 = wrist_position.z;
    let mut solutions = Vec::with_capacity(8);
    // a1
    let a1_1 = y.atan2(x);
    let mut a1_2 = a1_1 + PI;
    if a1_2 > TAU {
        a1_2 -= TAU;
    }
    // solve (a2, a3) with a1_1
    let x1 = x - K1 * a1_1.cos();
    let y1 = y - K1 * a1_1.sin();
    let square_xy = x1 * x1 + y1 * y1;
    let square_xyz = square_xy + z * z;
    let a3_acos = ((SQUARE_L1 + SQUARE_L2 - square_xyz) / (2.0 * L1 * L2)).acos();
    let gamma2 = ((SQUARE_L1 - SQUARE_L2 + square_xyz) / (2.0 * L1 * square_xyz.sqrt())).acos();
    if !a3_acos.is_nan() && !gamma2.is_nan() {
        let a2 = -gamma2 + square_xy.sqrt().atan2(z);
        let a3 = -GAMMA1 + a3_acos;
        let tf_3_6 = fk03(a1_1, a2, a3).inverse() * tf_0_6;
        let ((a4_1, a5_1, a6_1), (a4_2, a5_2, a6_2)) =
            solve_wrist_angles(&tf_3_6.rotation.to_rotation_matrix());
        solutions.push((a1_1, a2, a3, a4_1, a5_1, a6_1));
        solutions.push((a1_1, a2, a3, a4_2, a5_2, a6_2));

        let a2 = gamma2 + square_xy.sqrt().atan2(z);
        let a3 = -GAMMA1 - a3_acos + TAU;
        let tf_3_6 = fk03(a1_1, a2, a3).inverse() * tf_0_6;
        let ((a4_1, a5_1, a6_1), (a4_2, a5_2, a6_2)) =
            solve_wrist_angles(&tf_3_6.rotation.to_rotation_matrix());
        solutions.push((a1_1, a2, a3, a4_1, a5_1, a6_1));
        solutions.push((a1_1, a2, a3, a4_2, a5_2, a6_2));
    }
    // solve (a2, a3) with a1_2
    let x1 = x - K1 * a1_2.cos();
    let y1 = y - K1 * a1_2.sin();
    let square_xy = x1 * x1 + y1 * y1;
    let square_xyz = square_xy + z * z;
    let a3_acos = ((SQUARE_L1 + SQUARE_L2 - square_xyz) / (2.0 * L1 * L2)).acos();
    let gamma2 = ((SQUARE_L1 - SQUARE_L2 + square_xyz) / (2.0 * L1 * square_xyz.sqrt())).acos();
    if !a3_acos.is_nan() && !gamma2.is_nan() {
        let a2 = -gamma2 - square_xy.sqrt().atan2(z);
        let a3 = -GAMMA1 + a3_acos;
        let tf_3_6 = fk03(a1_2, a2, a3).inverse() * tf_0_6;
        let ((a4_1, a5_1, a6_1), (a4_2, a5_2, a6_2)) =
            solve_wrist_angles(&tf_3_6.rotation.to_rotation_matrix());
        solutions.push((a1_2, a2, a3, a4_1, a5_1, a6_1));
        solutions.push((a1_2, a2, a3, a4_2, a5_2, a6_2));

        let a2 = gamma2 - square_xy.sqrt().atan2(z);
        let a3 = -GAMMA1 - a3_acos + TAU;
        let tf_3_6 = fk03(a1_2, a2, a3).inverse() * tf_0_6;
        let ((a4_1, a5_1, a6_1), (a4_2, a5_2, a6_2)) =
            solve_wrist_angles(&tf_3_6.rotation.to_rotation_matrix());
        solutions.push((a1_2, a2, a3, a4_1, a5_1, a6_1));
        solutions.push((a1_2, a2, a3, a4_2, a5_2, a6_2));
    }
    solutions
}

#[rustfmt::skip]
pub fn fk0() -> na::Isometry3<f64> {
    let mat: na::Matrix4<f64> = na::Matrix4::new(
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.425,
                0.0, 0.0, 0.0, 1.0,
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

#[rustfmt::skip]
pub fn fk01(theta: f64) -> na::Isometry3<f64> {
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        theta.cos(), -theta.sin(), 0.0, 0.0,
        theta.sin(),  theta.cos(), 0.0, 0.0,
                0.0,          0.0, 1.0, 0.0,
                0.0,          0.0, 0.0, 1.0,
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

#[rustfmt::skip]
pub fn fk12(theta: f64) -> na::Isometry3<f64> {
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        theta.sin(),  theta.cos(), 0.0, 0.075,
                0.0,          0.0, 1.0,   0.0,
        theta.cos(), -theta.sin(), 0.0,   0.0,
                0.0,          0.0, 0.0,   1.0,
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

#[rustfmt::skip]
pub fn fk23(theta: f64) -> na::Isometry3<f64> {
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        theta.cos(), -theta.sin(),  0.0, 0.84,
       -theta.sin(), -theta.cos(),  0.0,  0.0,
                0.0,          0.0, -1.0,  0.0,
                0.0,          0.0,  0.0,  1.0,
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

#[rustfmt::skip]
pub fn fk34(theta: f64) -> na::Isometry3<f64> {
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        theta.cos(), -theta.sin(),  0.0, 0.215,
                0.0,          0.0,  1.0, -0.89,
       -theta.sin(), -theta.cos(),  0.0,   0.0,
                0.0,          0.0,  0.0,   1.0,
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

#[rustfmt::skip]
pub fn fk45(theta: f64) -> na::Isometry3<f64> {
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        theta.cos(), -theta.sin(),  0.0,   0.0,
                0.0,          0.0, -1.0,   0.0,
        theta.sin(),  theta.cos(),  0.0,   0.0,
                0.0,          0.0,  0.0,   1.0,
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

#[rustfmt::skip]
pub fn fk56(theta: f64) -> na::Isometry3<f64> {
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        theta.cos(), -theta.sin(),  0.0,   0.0,
                0.0,          0.0,  1.0,   0.0,
       -theta.sin(), -theta.cos(),  0.0,   0.0,
                0.0,          0.0,  0.0,   1.0,
    );
    na::Isometry3::from_superset_unchecked(&mat)
}

#[rustfmt::skip]
pub fn fk6f() -> na::Isometry3<f64> {
    let mat: na::Matrix4<f64> = na::Matrix4::new(
        1.0,  0.0,  0.0,   0.0,
        0.0, -1.0,  0.0,   0.0,
        0.0,  0.0, -1.0, -0.09,
        0.0,  0.0,  0.0,   1.0
    );
    na::Isometry3::from_superset_unchecked(&mat)
}
