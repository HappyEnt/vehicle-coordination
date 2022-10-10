extern crate coordination;

use std::{collections::HashMap, env, error::Error, thread, time::Duration};

use api::{ApiPayload, ParticipantInformation};
use coordination::{
    interface::{
        coordination_client::CoordinationClient, tick_request::Participant, TickRequest, Vec2,
    },
    math::signed_angle_between,
    util::parse_cmd_arg,
};
use log::{debug, error, warn};
use orca_rs::ndarray::{arr1, Array1};

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

type Coord = [f64; 2];

const TARGETS: [Coord; 2] = [
    //
    [0.2, 0.2],
    // [0.2, 1.8],
    [1.4, 1.8],
    // [1.4, 0.2],
];

#[derive(Clone, Copy)]
enum Target {
    First,
    Second,
    Third,
    Fourth,
}

struct TargetWrapper {
    target: Target,
}

/// Wrapper around dynamic target selection
impl TargetWrapper {
    pub fn new() -> Self {
        TargetWrapper {
            target: Target::First,
        }
    }

    pub fn next(&mut self) {
        self.target = match self.target {
            Target::First => Target::Second,
            Target::Second => Target::First,
            Target::Third => Target::Fourth,
            Target::Fourth => Target::First,
        }
    }

    pub fn usize(&self) -> usize {
        match self.target {
            Target::First => 0,
            Target::Second => 1,
            Target::Third => 2,
            Target::Fourth => 3,
        }
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
    // TODO: make this address a cmd arg
    #[cfg(not(feature = "pi"))]
    let address = "http://192.168.1.101:50052";
    #[cfg(feature = "pi")]
    let address = "http://0.0.0.0:50052";

    let args: Vec<String> = env::args().collect();
    let id = parse_cmd_arg(args.get(1).expect("No ID given!"));

    let camera = CameraInterface::new("http://192.168.87.78:8081/positions", id);
    let mut target_wrapper = TargetWrapper::new();
    let mut client = CoordinationClient::connect(address.to_owned()).await?;

    let mut old_position: Option<Array1<f64>> = None;
    let mut last_direction: Option<Array1<f64>>;
    let mut desired_direction: Option<Array1<f64>> = None;

    loop {
        let payload = camera.fetch().await?;

        if let Some(new_position) = payload.position {
            // update information about the last direction and the current position
            last_direction = calculate_position_delta(&old_position, &new_position);
            old_position = Some(new_position.clone());
            diff_desired_and_actual_direction(&desired_direction, &last_direction);

            // update the target of the car according to current wrapper
            let target: [f64; 2] = TARGETS[target_wrapper.usize()];
            if let Err(e) = client.set_target(Vec2::from_pos(&arr1(&target))).await {
                error!("Error setting new target: {:#?}", e);
            }

            let tick_request = TickRequest {
                id: 6,
                position: Some(Vec2::from_pos(&new_position)),
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
                obstacles: vec![],
            };

            // check if the car thinks it is finished
            let response = client.tick(tick_request).await?;
            let response = response.into_inner();
            if response.finished {
                target_wrapper.next();
            }

            // update the stored "desired velocity"
            if let Some(new_velocity) = response.new_velocity {
                desired_direction = Some(new_velocity.to_pos());
            }
        }

        thread::sleep(Duration::from_millis(SLEEP_TIMER));
    }
}

fn calculate_position_delta(
    old_position: &Option<Array1<f64>>,
    new_position: &Array1<f64>,
) -> Option<Array1<f64>> {
    debug!(
        "calculate_position_delta({:#?}, {:#?})",
        old_position, new_position
    );
    old_position
        .clone()
        .map(|old_position| new_position - old_position)
}

fn diff_desired_and_actual_direction(
    desired_direction: &Option<Array1<f64>>,
    actual_direction: &Option<Array1<f64>>,
) {
    debug!(
        "diff_desired_and_actual_direction({:#?}, {:#?})",
        desired_direction, actual_direction
    );

    if let (Some(desired_direction), Some(actual_direction)) = (desired_direction, actual_direction)
    {
        println!(
            "{}",
            signed_angle_between(desired_direction, actual_direction)
        );
    } else {
        warn!(
            "Trying to calculate the difference between {:#?} and {:#?}",
            desired_direction, actual_direction
        );
    }
}
