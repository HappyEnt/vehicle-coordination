extern crate coordination;

use coordination::{
    interface::{
        coordination_server::{Coordination, CoordinationServer},
        TickRequest, TickResponse, Vec2,
    },
    orca_car::OrcaCar,
};
use log::{debug, error};
use orca_rs::ndarray::arr1;
use orca_rs::participant::Participant;
use std::{collections::HashMap, error::Error};
use tokio::sync::Mutex;
use tonic::{transport::Server, Response, Status};

struct CoordinationService {
    car: Mutex<OrcaCar>,
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
        let mut car = self.car.lock().await;
        car.update(request.clone().position.unwrap().to_pos(), None);

        let participants = self.get_participants(request.clone()).await;
        let new_velocity = car
            .tick(
                participants,
                request.radius as f64,
                request.confidence as f64,
            )
            .await;

        match new_velocity {
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
    async fn get_participants(&self, request: TickRequest) -> Vec<Participant> {
        let others = request.others;
        let mut participants = self.participants.lock().await;

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
                        Participant::new(
                            other.position.unwrap().to_pos(),
                            arr1(&[0.0, 0.0]),
                            other.radius as f64,
                            other.confidence as f64,
                        ),
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

    let car = OrcaCar::new(None, Some(arr1(&[0.2, 0.2]))).await?;

    let coordination_service = CoordinationService {
        car: Mutex::new(car),
        participants: Mutex::new(HashMap::new()),
    };

    debug!("Attempting to start server...");
    Server::builder()
        .add_service(CoordinationServer::new(coordination_service))
        .serve(addr)
        .await?;
    Ok(())
}
