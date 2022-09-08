extern crate coordination;

use std::{collections::HashMap, env, error::Error, thread, time::Duration};

use api::{ApiPayload, ParticipantInformation};
use coordination::{
    interface::{
        coordination_client::CoordinationClient, tick_request::Participant, TickRequest, Vec2,
    },
    util::parse_cmd_arg,
};
use orca_rs::ndarray::arr1;

const SLEEP_TIMER: u64 = 100;

const CAR_RADIUS: f64 = 0.15;

/// All API types comming from the camera server.
mod api {
    use orca_rs::ndarray::Array1;

    /// Payload received via the API.
    #[derive(Clone, Debug)]
    pub struct ApiPayload {
        /// ID of the participant receiving the payload
        pub id: u8,
        /// Position of the particiapant receiving the payload
        pub position: Option<Array1<f64>>,
        /// Radius of the participant receiving the payload
        pub radius: f64,
        /// Inaccuracy in term of the position of the participant receiving the payload
        pub inaccuracy: f64,
        /// Information about other participants in the system
        pub others: Vec<ParticipantInformation>,
    }

    /// Information about a single other participant.
    #[derive(Clone, Debug)]
    pub struct ParticipantInformation {
        /// ID of this participant
        pub id: u8,
        /// Current position of this participant
        pub position: Array1<f64>,
        /// Radius of this participant
        pub radius: f64,
        /// Inaccuracy in terms of the position of this participant
        pub inaccuracy: f64,
    }
}

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
                radius: CAR_RADIUS,
                inaccuracy: 0.0,
            })
        }

        Ok(ApiPayload {
            id: self.id,
            position: own_position,
            radius: CAR_RADIUS,
            inaccuracy: 0.0,
            others,
        })
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    #[cfg(not(feature = "pi"))]
    let address = "http://192.168.1.101:50052";
    #[cfg(feature = "pi")]
    let address = "http://0.0.0.0:50052";

    let args: Vec<String> = env::args().collect();
    let id = parse_cmd_arg(args.get(1).expect("No ID given!"));

    let camera = CameraInterface::new("http://192.168.87.78:8081/positions", id);
    let mut client = CoordinationClient::connect(address.to_owned()).await?;

    loop {
        let payload = camera.fetch().await?;

        if let Some(position) = payload.position {
            let tick_request = TickRequest {
                id: 6,
                position: Some(Vec2::from_pos(&position)),
                confidence: 0.0,
                radius: 0.2,
                others: payload
                    .others
                    .iter()
                    .map(|p| Participant {
                        id: p.id as i32,
                        position: Some(Vec2::from_pos(&p.position)),
                        confidence: 0.0,
                        radius: p.radius as f32,
                    })
                    .collect(),
            };

            client.tick(tick_request).await?;
        }

        thread::sleep(Duration::from_millis(SLEEP_TIMER));
    }
}