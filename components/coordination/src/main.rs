mod api;
mod camera_interface;
mod math;
mod orca_car;
mod picar;
mod util;
mod wheels;

use coordination::{
    coordination_server::{Coordination, CoordinationServer},
    TickRequest, TickResponse, Vec2,
};
use log::{debug, error};
use ndarray::arr1;
use orca_car::OrcaCar;
use orca_rs::participant::Participant;
use std::{collections::HashMap, error::Error, sync::Mutex};
use tonic::{transport::Server, Response, Status};

pub mod coordination {
    use ndarray::{arr1, Array1};

    tonic::include_proto!("coordination");

    impl Vec2 {
        pub fn to_pos(&self) -> Array1<f64> {
            arr1(&[self.x as f64, self.y as f64])
        }
    }
}

struct CoordinationService {
    car: OrcaCar,
    participants: Mutex<HashMap<i32, Participant>>,
}

#[tonic::async_trait]
impl Coordination for CoordinationService {
    async fn tick(
        &self,
        request: tonic::Request<TickRequest>,
    ) -> Result<Response<TickResponse>, Status> {
        debug!("Coordination::tick({:?})", request);
        let request = request.into_inner().clone();
        let mut car = self.car.clone();
        car.update(request.clone().position.unwrap().to_pos(), None);

        let participants = self.get_participants(request.clone());
        let new_velocity = car.tick(
            participants,
            request.radius as f64,
            request.confidence as f64,
        );
        let next_vel = new_velocity.await;

        match next_vel {
            Ok(new_vel) => {
                return Ok(Response::new(TickResponse {
                    new_velocity: new_vel.map(|new_vel| Vec2 {
                        x: new_vel[0] as f32,
                        y: new_vel[1] as f32,
                    }),
                }));
            }
            Err(e) => {
                error!("Error during tick: ({})", e);
                return Err(Status::internal("Error during calculation of new velocity"));
            }
        }
    }
}

impl CoordinationService {
    /// Get the information about other participants.
    fn get_participants(&self, request: TickRequest) -> Vec<Participant> {
        let others = request.others;
        let mut participants = self.participants.lock().unwrap();

        for other in others {
            // try to update (or insert) information about other participants
            match participants.get_mut(&other.id) {
                Some(participant) => {
                    participant.update_position(&other.position.unwrap().to_pos());
                    participant.radius = other.radius as f64;
                    participant.confidence = other.confidence as f64;
                }
                None => {
                    participants.insert(
                        other.id,
                        Participant {
                            position: other.position.unwrap().to_pos(),
                            vmax: 0.0,
                            confidence: other.confidence as f64,
                            radius: other.radius as f64,
                            target: arr1(&[0.0, 0.0]),
                            velocity: arr1(&[0.0, 0.0]),
                            in_obstacle: false,
                        },
                    );
                }
            }
        }
        participants.clone().into_values().collect()
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let addr = "0.0.0.0:50052".parse()?;

    let car = OrcaCar::new(None, None).await?;

    let coordination_service = CoordinationService {
        car,
        participants: Mutex::new(HashMap::new()),
    };

    debug!("Attempting to start server...");
    Server::builder()
        .add_service(CoordinationServer::new(coordination_service))
        .serve(addr)
        .await?;
    Ok(())
}
