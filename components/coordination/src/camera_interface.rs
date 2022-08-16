use std::{collections::HashMap, error::Error};

use crate::api::{ApiPayload, ParticipantInformation};

pub struct CameraInterface {
    url: String,
    id: u8,
}

pub trait Callback: Sized {
    fn handle_payload(&self, payload: ApiPayload);
}

impl<F> Callback for F
where
    F: Fn(ApiPayload) -> (),
{
    fn handle_payload(&self, payload: ApiPayload) {
        self(payload)
    }
}

impl CameraInterface {
    pub fn new(url: &str, id: u8) -> Self {
        CameraInterface {
            url: url.to_owned(),
            id,
        }
    }

    /// Fetch the current information from the camera API.
    pub async fn fetch(&self) -> Result<ApiPayload, Box<dyn Error>> {
        let cur_positions = reqwest::get(self.url.clone())
            .await?
            .json::<HashMap<String, (f64, f64)>>()
            .await?;
        // debug!("Fetched: {:?}", cur_positions);

        let own_position = match cur_positions.get(&self.id.to_string()) {
            None => None,
            Some((x, y)) => Some(ndarray::arr1(&[*x, *y])),
        };

        let mut others: Vec<ParticipantInformation> = vec![];

        for key in cur_positions.keys() {
            if key == &self.id.to_string() {
                continue;
            }

            let (x, y) = cur_positions.get(key).unwrap();
            others.push(ParticipantInformation {
                id: key.parse::<u8>()?,
                position: ndarray::arr1(&[*x, *y]),
                radius: 0.0,
                inaccuracy: 0.0,
            })
        }

        Ok(ApiPayload {
            id: self.id,
            position: own_position,
            radius: 0.0,
            inaccuracy: 0.0,
            others,
        })
    }
}
