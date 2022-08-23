extern crate coordination;

use std::{collections::HashMap, error::Error, thread, time::Duration};

use coordination::{
    api::{ApiPayload, ParticipantInformation},
    interface::{coordination_client::CoordinationClient, TickRequest, Vec2},
};
use orca_rs::ndarray::arr1;

const SLEEP_TIMER: u64 = 1000;

pub struct CameraInterface {
    url: String,
    id: u8,
}

pub trait Callback: Sized {
    fn handle_payload(&self, payload: ApiPayload);
}

impl<F> Callback for F
where
    F: Fn(ApiPayload),
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

        let own_position = cur_positions
            .get(&self.id.to_string())
            .map(|(x, y)| arr1(&[*x, *y]));

        let mut others: Vec<ParticipantInformation> = vec![];

        for key in cur_positions.keys() {
            if key == &self.id.to_string() {
                continue;
            }

            let (x, y) = cur_positions.get(key).unwrap();
            others.push(ParticipantInformation {
                id: key.parse::<u8>()?,
                position: arr1(&[*x, *y]),
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

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    #[cfg(not(feature = "pi"))]
    let address = "http://192.168.1.101:50052";
    #[cfg(feature = "pi")]
    let address = "http://0.0.0.0:50052";

    let camera = CameraInterface::new("http://192.168.87.78:8081/positions", 6);
    let mut client = CoordinationClient::connect(address.to_owned()).await?;

    loop {
        let payload = camera.fetch().await?;

        let tick_request = TickRequest {
            id: 6,
            position: payload.position.map(|pos| Vec2::from_pos(&pos)),
            confidence: 0.0,
            radius: 7.0,
            others: vec![],
        };

        client.tick(tick_request).await?;

        thread::sleep(Duration::from_millis(SLEEP_TIMER));
    }
}
