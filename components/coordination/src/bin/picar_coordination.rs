extern crate coordination;

use coordination::{
    interface::{
        coordination_server::{Coordination, CoordinationServer},
        Empty, TickRequest, TickResponse, Vec2,
    },
    orca_car::OrcaCar,
    util::parse_cmd_arg,
};
use log::{debug, error, warn};
use orca_rs::participant::Participant;
use orca_rs::{ndarray::arr1, obstacle::Obstacle};
use std::{collections::HashMap, env, error::Error};
use tokio::sync::Mutex;
use tonic::{transport::Server, Response, Status};

struct CoordinationService {
    car: Mutex<OrcaCar>,
    participants: Mutex<HashMap<i32, Participant>>,
}

static DEFAULT_PORT: &str = "50052";

#[tonic::async_trait]
impl Coordination for CoordinationService {
    async fn tick(
        &self,
        request: tonic::Request<TickRequest>,
    ) -> Result<Response<TickResponse>, Status> {
        debug!("Coordination::tick({:?})", request);
        let request = request.into_inner().clone();
        let mut car = self.car.lock().await;

        // FIXME: This panics, if position is None
        car.update_position(request.clone().position.unwrap().to_pos());

        let participants = self.get_participants(request.clone()).await;
        let obstacles = self.get_obstacles(request.clone());
        let new_velocity = car
            .tick(
                participants,
                obstacles,
                request.radius as f64,
                request.confidence as f64,
            )
            .await;

        match new_velocity {
            Ok((new_vel, finished)) => {
                return Ok(Response::new(TickResponse {
                    new_velocity: new_vel.map(|new_vel| Vec2 {
                        x: new_vel[0] as f32,
                        y: new_vel[1] as f32,
                    }),
                    finished,
                }));
            }
            Err(e) => {
                error!("Error during tick: ({})", e);
                return Err(Status::internal("Error during calculation of new velocity"));
            }
        }
    }

    /// Set the target of the car.
    async fn set_target(&self, request: tonic::Request<Vec2>) -> Result<Response<Empty>, Status> {
        debug!("Coordination::set_target({:?})", request);
        let target = request.into_inner();
        let mut car = self.car.lock().await;
        car.update_target(target.to_pos());

        Ok(Response::new(Empty {}))
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
                    // TODO: integrate this into orca
                    participant.velocity =
                        &other.position.as_ref().unwrap().to_pos() - &participant.position;
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

    /// Get obstacles from a tick request.
    fn get_obstacles(&self, request: TickRequest) -> Vec<Obstacle> {
        request
            .obstacles
            .into_iter()
            .map(|obstacle| Obstacle {
                start: obstacle
                    .start
                    .expect("obstacle.start is undefined")
                    .to_pos(),
                end: obstacle.end.expect("obstacle.end is undefined").to_pos(),
                radius: obstacle.radius as f64,
            })
            .collect()
    }
}

#[cfg(all(feature = "pi", feature = "simulation"))]
compile_error!("feature \"pi\" and feature \"simulation\" cannot be enabled at the same time");

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args: Vec<String> = env::args().collect();
    let port: u16 = parse_cmd_arg(&match args.get(1) {
        Some(port) => port.to_owned(),
        None => {
            warn!("No port provided! Falling back to :{}", DEFAULT_PORT);
            DEFAULT_PORT.to_owned()
        }
    });

    let addr = format!("0.0.0.0:{}", port).parse()?;

    let car = OrcaCar::new(None, None, port - 1).await?;

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
