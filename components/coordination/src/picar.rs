//! A light wrapper around the wheel control of the car (i.e., driving, turning, etc.)

use crate::wheels::Wheels;
use log::debug;
use std::error::Error;

/// The speed for the wheels to turn with.
const TURN_SPEED: f64 = 0.1;

/// The speed for the wheels to drive.
const DRIVE_SPEED: f64 = 0.2;

/// Possible directions to turn.
#[derive(Clone, Debug)]
enum Direction {
    Left = 1,
    Right = -1,
}

/// Configuration for the car. This includes the maximum speed and the amount a car can turn.  
#[derive(Default, Clone, Copy)]
pub struct PicarConfig {
    /// Maximum speed of the car.
    pub max_speed: f64,
    /// Amount this car can turn to the right.
    pub right_turn: f64,
    /// Amount this car can turn to the left.
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
    /// Construct a new instance which directly connects to the wheels.
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

    /// Start driving forward.
    pub async fn drive(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("Picar::drive()");
        self.wheels.set_speed(DRIVE_SPEED, DRIVE_SPEED).await
    }

    /// Stop the wheels of the car.
    pub async fn stop(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("Picar::stop()");
        self.wheels.set_speed(0.0, 0.0).await
    }

    /// Start turning left.
    pub async fn turn_left(&mut self) -> Result<(), Box<dyn Error>> {
        self.turn(Direction::Left).await
    }

    /// Start turning right.
    pub async fn turn_right(&mut self) -> Result<(), Box<dyn Error>> {
        self.turn(Direction::Right).await
    }

    /// Start to turn in the given direction.
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
