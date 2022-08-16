use crate::wheels::Wheels;
use log::debug;
use std::{error::Error, time::Duration};

const TURN_DURATION: u64 = 1000;
const TURN_SPEED: f64 = 0.1;

const DRIVE_DURATION: u64 = 1000;
const DRIVE_SPEED: f64 = 0.2;

#[derive(Clone, Debug)]
enum Direction {
    Left = 1,
    Right = -1,
}

#[derive(Default, Clone, Copy)]
pub struct PicarConfig {
    pub max_speed: f64,
    pub right_turn: f64,
    pub left_turn: f64,
}

/// An abstraction around the actual wheel control of the car.
/// This features simple actions like driving forward, turning left and right,
/// and stopping.
#[derive(Clone)]
pub struct Picar {
    wheels: Wheels,
}

impl Picar {
    pub async fn new() -> Self {
        debug!("Picar::new()");
        Self {
            #[cfg(not(feature = "pi"))]
            // TODO: This should be a command line arg
            wheels: Wheels::from_address("http://192.168.1.101:50051").await.unwrap(),
            #[cfg(feature = "pi")]
            wheels: Wheels::new().await.unwrap(),
        }
    }

    pub async fn drive(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("Picar::drive()");
        self.wheels.set_speed(DRIVE_SPEED, DRIVE_SPEED).await
    }

    pub async fn stop(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("Picar::stop()");
        self.wheels.set_speed(0.0, 0.0).await
    }

    pub async fn turn_left(&mut self) -> Result<(), Box<dyn Error>> {
        self.turn(Direction::Left).await
    }

    pub async fn turn_right(&mut self) -> Result<(), Box<dyn Error>> {
        self.turn(Direction::Right).await
    }

    async fn turn(&mut self, direction: Direction) -> Result<(), Box<dyn Error>> {
        debug!("Picar::turn({:?})", &direction);
        self.wheels
            .set_speed(
                -TURN_SPEED * direction.clone() as i64 as f64,
                TURN_SPEED * direction.clone() as i64 as f64,
            )
            .await?;
        Ok(())
    }
}
