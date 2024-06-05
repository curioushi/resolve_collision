use ncollide3d::na;
use std::io::{BufRead, Read, Write};

pub fn write_pcd(points: &Vec<na::Point3<f64>>, path: &str) {
    let mut file = std::fs::File::create(path).unwrap();
    file.write_all(b"VERSION .7\n").unwrap();
    file.write_all(b"FIELDS x y z\n").unwrap();
    file.write_all(b"SIZE 4 4 4\n").unwrap();
    file.write_all(b"TYPE F F F\n").unwrap();
    file.write_all(b"COUNT 1 1 1\n").unwrap();
    file.write_all(b"WIDTH ").unwrap();
    file.write_all(format!("{}\n", points.len()).as_bytes())
        .unwrap();
    file.write_all(b"HEIGHT 1\n").unwrap();
    file.write_all(b"VIEWPOINT 0 0 0 1 0 0 0\n").unwrap();
    file.write_all(b"POINTS ").unwrap();
    file.write_all(format!("{}\n", points.len()).as_bytes())
        .unwrap();
    file.write_all(b"DATA binary\n").unwrap();
    for i in 0..points.len() {
        file.write_all(&(points[i].coords.x as f32).to_le_bytes())
            .unwrap();
        file.write_all(&(points[i].coords.y as f32).to_le_bytes())
            .unwrap();
        file.write_all(&(points[i].coords.z as f32).to_le_bytes())
            .unwrap();
    }
}

pub fn write_pcd_with_normal(
    points: &Vec<na::Point3<f64>>,
    normals: &Vec<na::Vector3<f64>>,
    path: &str,
) {
    let mut file = std::fs::File::create(format!("{}/cloud.pcd", path)).unwrap();
    file.write_all(b"VERSION .7\n").unwrap();
    file.write_all(b"FIELDS x y z normal_x normal_y normal_z\n")
        .unwrap();
    file.write_all(b"SIZE 4 4 4 4 4 4\n").unwrap();
    file.write_all(b"TYPE F F F F F F\n").unwrap();
    file.write_all(b"COUNT 1 1 1 1 1 1\n").unwrap();
    file.write_all(b"WIDTH ").unwrap();
    file.write_all(format!("{}\n", points.len()).as_bytes())
        .unwrap();
    file.write_all(b"HEIGHT 1\n").unwrap();
    file.write_all(b"VIEWPOINT 0 0 0 1 0 0 0\n").unwrap();
    file.write_all(b"POINTS ").unwrap();
    file.write_all(format!("{}\n", points.len()).as_bytes())
        .unwrap();
    file.write_all(b"DATA binary\n").unwrap();
    for i in 0..points.len() {
        file.write_all(&(points[i].coords.x as f32).to_le_bytes())
            .unwrap();
        file.write_all(&(points[i].coords.y as f32).to_le_bytes())
            .unwrap();
        file.write_all(&(points[i].coords.z as f32).to_le_bytes())
            .unwrap();
        file.write_all(&(normals[i].x as f32).to_le_bytes())
            .unwrap();
        file.write_all(&(normals[i].y as f32).to_le_bytes())
            .unwrap();
        file.write_all(&(normals[i].z as f32).to_le_bytes())
            .unwrap();
    }
}

pub fn read_pcd(filepath: &str) -> Vec<na::Point3<f64>> {
    let file = std::fs::File::open(filepath).unwrap();
    let mut reader = std::io::BufReader::new(file);
    let mut has_normal = false;
    // read until the end of the header
    loop {
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();
        if line.trim().contains("normal_x") {
            has_normal = true;
        }
        if line.trim() == "DATA binary" {
            break;
        }
    }
    // read the binary data
    let mut pc: Vec<na::Point3<f64>> = vec![];
    loop {
        match has_normal {
            true => {
                let mut buf = [0u8; 4 * 6];
                match reader.read_exact(&mut buf) {
                    Ok(_) => {
                        let x = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                        let y = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
                        let z = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
                        pc.push(na::Point3::new(x as f64, y as f64, z as f64));
                    }
                    Err(_) => break,
                }
            }
            false => {
                let mut buf = [0u8; 4 * 3];
                match reader.read_exact(&mut buf) {
                    Ok(_) => {
                        let x = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                        let y = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
                        let z = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
                        pc.push(na::Point3::new(x as f64, y as f64, z as f64));
                    }
                    Err(_) => break,
                }
            }
        }
    }
    pc
}
