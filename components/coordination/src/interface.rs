use orca_rs::ndarray::{arr1, Array1};

tonic::include_proto!("coordination");

impl Vec2 {
    pub fn to_pos(&self) -> Array1<f64> {
        arr1(&[self.x as f64, self.y as f64])
    }

    pub fn from_pos(pos: &Array1<f64>) -> Self {
        Self {
            x: pos[0] as f32,
            y: pos[1] as f32,
        }
    }
}
