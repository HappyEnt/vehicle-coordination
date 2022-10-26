//! Wrapper around Picar containing calibration and ORCA.

use std::{error::Error, f64::consts::PI, time::Duration};

use async_channel::{Receiver, Sender};
use log::{debug, error, warn};
use orca_rs::{
    ndarray::{arr1, Array1},
    obstacle::Obstacle,
    participant::Participant,
};
use tokio::time::timeout;

use crate::{
    math::{dist, signed_angle_between},
    wheels::Wheels,
};

type NewVelAdjustment = (Option<Array1<f64>>, Option<Array1<f64>>);

/// The speed for the wheels to drive.
const DRIVE_SPEED: f64 = 0.8;

const MAX_SPEED: f64 = DRIVE_SPEED;

/// Timeout after which the car stops driving.
const TIMEOUT: u64 = 2000;
const DISTANCE_THRESHOLD: f64 = 0.2;

/// A wrapper around the ORCA algorithm "on a car".
/// This includes an abstraction of the underlying car (i.e., the wheels),
/// calibration, movement, and the execution of ORCA.
#[derive(Clone)]
pub struct OrcaCar {
    position: Option<Array1<f64>>,
    target: Option<Array1<f64>>,
    old_velocity: Option<Array1<f64>>,
    velocity: Option<Array1<f64>>,
    channel: (Sender<NewVelAdjustment>, Receiver<NewVelAdjustment>),
    _after_update: bool,
}

impl OrcaCar {
    /// Create a new instance of an OrcaCar
    pub async fn new(
        position: Option<Array1<f64>>,
        target: Option<Array1<f64>>,
        port: u16,
    ) -> Result<Self, Box<dyn Error>> {
        debug!("OrcaCar::new({:?}, {:?})", position, target);
        debug!("Successfully created OrcaCar!");
        let instance = Self {
            position,
            target,
            old_velocity: None,
            velocity: None,
            channel: async_channel::unbounded::<NewVelAdjustment>(),
            _after_update: false,
        };

        let (_, receiver) = instance.channel.clone();
        tokio::spawn(async move {
            #[cfg(not(any(feature = "pi", feature = "simulation")))]
            let mut wheels =
                Wheels::from_address(format!("http://192.168.1.101:{}", port).as_str())
                    .await
                    .unwrap();
            #[cfg(any(feature = "pi", feature = "simulation"))]
            let mut wheels = Wheels::new(port).await.unwrap();
            loop {
                // try to fetch new direction
                debug!("Trying to receive from sender...");
                match timeout(Duration::from_millis(TIMEOUT), receiver.recv()).await {
                    Ok(res) => {
                        debug!("Received {:?}", res);
                        match res {
                            Ok((None, _)) => {
                                if let Err(e) = wheels.set_speed(DRIVE_SPEED, DRIVE_SPEED).await {
                                    error!("Error setting speed of wheels: {}", e);
                                }
                            }
                            Ok((_, None)) => {
                                if let Err(e) = wheels.set_speed(0.0, 0.0).await {
                                    error!("Error setting speed of wheels: {}", e);
                                }
                            }
                            Ok((Some(old_vel), Some(new_vel))) => {
                                // let velocity_scalar = norm(&new_vel) / MAX_SPEED;
                                let velocity_scalar = 1.0;
                                // check, if we have to turn right or left
                                let angle = signed_angle_between(&old_vel, &new_vel);
                                let diff = angle.abs() as f64 / PI;
                                // TODO: Maybe to sth like x^2 or x^3
                                let factor = 2.0 * diff + 1.0;

                                let mut right_factor = DRIVE_SPEED * velocity_scalar;
                                let mut left_factor = DRIVE_SPEED * velocity_scalar;

                                if diff.is_normal() {
                                    if angle.is_sign_positive() {
                                        // to the right
                                        left_factor *= factor;
                                        right_factor /= factor;
                                    } else {
                                        // to the left
                                        left_factor /= factor;
                                        right_factor *= factor;
                                    }
                                }
                                if let Err(e) = wheels.set_speed(left_factor, right_factor).await {
                                    error!("Error setting speed of wheels: {}", e);
                                }
                            }
                            Err(e) => {
                                error!("Error during reading from channel: {}", e);
                                break;
                            }
                        }
                    }
                    Err(_) => {
                        warn!("Not received anything for {}ms", TIMEOUT);

                        if let Err(e) = wheels.set_speed(0.0, 0.0).await {
                            error!("Error during picar.stop() ({})", e);
                        }
                    }
                }
            }
        });
        Ok(instance)
    }

    /// Update the information about this car.
    /// Note: This should be called each time before calling OrcaCar::tick!
    pub fn update_position(&mut self, position: Array1<f64>) {
        debug!("OrcaCar::update({})", position);
        if self._after_update {
            warn!("Calling OrcaCar::update() twice without calling OrcaCar::tick()!");
        }
        self.old_velocity = self.velocity.clone();
        debug!("self.old_velocity = {:?}", self.old_velocity);
        self.velocity = Some(
            &position
                - match self.position.clone() {
                    None => arr1(&[0.0, 0.0]),
                    Some(pos) => pos,
                },
        );
        debug!("self.velocity = {:?}", self.velocity);
        self.position = Some(position);
        debug!("self.position = {:?}", self.position);
        self._after_update = true;
    }

    /// Update the target of this car.
    pub fn update_target(&mut self, target: Array1<f64>) {
        self.target = Some(target);
    }

    /// Perform an actual tick of this car.
    /// This is either a calibration tick or an actual tick of the ORCA algorithm depending on the
    /// calibration stage of this car.
    ///
    /// Returns the new "desired" velocity, or None if still in calibration.
    ///
    /// Note: You should call OrcaCar::update each time before calling this function!
    pub async fn tick(
        &mut self,
        others: Vec<Participant>,
        obstacles: Vec<Obstacle>,
        radius: f64,
        confidence: f64,
    ) -> Result<(Option<Array1<f64>>, bool), Box<dyn Error>> {
        debug!("OrcaCar::tick({:?}, {}, {})", others, radius, confidence);
        if !self._after_update {
            warn!("Calling OrcaCar::tick() without calling OrcaCar::update() before!");
        }
        self._after_update = false;
        self.orca_tick(others, obstacles, radius, confidence).await
    }

    async fn orca_tick(
        &mut self,
        mut others: Vec<Participant>,
        obstacles: Vec<Obstacle>,
        radius: f64,
        confidence: f64,
    ) -> Result<(Option<Array1<f64>>, bool), Box<dyn Error>> {
        debug!(
            "OrcaCar::orca_tick({:?}, {}, {})",
            others, radius, confidence
        );
        let (sender, _) = self.channel.clone();

        let old_velocity = self.velocity.clone();
        // TODO: Do we actually have them all the time?
        // stop car if we are near enough to our target
        if dist(
            &self.position.clone().unwrap(),
            &self.target.clone().unwrap(),
        ) < DISTANCE_THRESHOLD
        {
            sender.send((old_velocity, None)).await?;
            return Ok((None, true));
        }

        let mut new_vel = arr1(&[0.0, 0.0]);

        if let (Some(position), Some(target)) = (self.position.clone(), self.target.clone()) {
            let mut we = Participant::new(position.clone(), arr1(&[0.0, 0.0]), radius, confidence)
                .with_inner_state(MAX_SPEED, target);
            we.update_position(&position);
            new_vel = we.orca(&mut others, &obstacles, 20.0);
        }

        debug!("Sending ({:?}, {})", old_velocity, new_vel);
        sender.send((old_velocity, Some(new_vel.clone()))).await?;
        Ok((Some(new_vel), false))
    }
}
