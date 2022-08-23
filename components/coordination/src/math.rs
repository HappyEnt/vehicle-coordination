use orca_rs::ndarray::{arr1, Array1};

pub fn norm(vector: &Array1<f64>) -> f64 {
    vector.dot(vector).sqrt()
}

pub fn dist(x: &Array1<f64>, y: &Array1<f64>) -> f64 {
    norm(&(x - y))
}

pub fn normalize(vec: &Array1<f64>) -> Array1<f64> {
    let len = norm(vec);
    if len == 0.0 {
        return arr1(&[0.0, 0.0]);
    }
    vec / len
}

fn rotate_by(vec: &Array1<f64>, angle: f64) -> Array1<f64> {
    let cos = angle.cos();
    let sin = angle.sin();
    let x = vec[0] * cos - vec[1] * sin;
    let y = vec[0] * sin + vec[1] * cos;
    arr1(&[x, y])
}

fn angle_between(a: &Array1<f64>, b: &Array1<f64>) -> f64 {
    let a_x = &a[0];
    let a_y = &a[1];
    let b_x = &b[0];
    let b_y = &b[1];
    ((a_x * b_x + a_y * b_y) / ((a_x * a_x + a_y * a_y).sqrt() * (b_x * b_x + b_y * b_y).sqrt()))
        .acos()
}

pub fn angle_to_base(a: &Array1<f64>) -> f64 {
    a[0].signum() * angle_between(a, &arr1(&[0.0, 1.0]))
}

pub fn signed_angle_between(a: &Array1<f64>, b: &Array1<f64>) -> f64 {
    let rotation_angle = angle_to_base(a);
    let rotated = rotate_by(b, rotation_angle);
    angle_between(a, b) * rotated[0].signum() * -1.0
}
