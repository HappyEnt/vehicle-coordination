//! Wrapper around Picar containing calibration and ORCA.

use std::{error::Error, thread, time::Duration};

use async_channel::{Receiver, Sender};
use log::{debug, error, warn};
use orca_rs::{
    ndarray::{arr1, Array1},
    participant::Participant,
};
use tokio::time::timeout;

use crate::{
    math::{dist, norm, signed_angle_between},
    picar::{Picar, PicarConfig},
};

type NewVelAdjustment = (Array1<f64>, Array1<f64>);

/// Duration to measure the base turn speed.
const TURN_DURATION: u64 = 1000;

/// Duration to measure the base drive speed.
const DRIVE_DURATION: u64 = 1000;

/// Timeout after which the car stops driving.
const TIMEOUT: u64 = 2000;
const DISTANCE_THRESHOLD: f64 = 0.2;

/// Different calibration stages for the car.
#[derive(Clone)]
enum CalibrationStage {
    /// Calibration has not started yet.
    No = 0,
    /// Car is during calibration of the maximum speed.
    DuringMaxSpeed = 1,
    /// Car is during calibration of the maximum right turn "amount".
    DuringRightTurn = 2,
    /// Car is during calibration of the maximum left turn "amount".
    DuringLeftTurn = 3,
    /// Calibration is done.
    Done = 4,
}

/// A wrapper around the ORCA algorithm "on a car".
/// This includes an abstraction of the underlying car (i.e., the wheels),
/// calibration, movement, and the execution of ORCA.
#[derive(Clone)]
pub struct OrcaCar {
    position: Option<Array1<f64>>,
    target: Option<Array1<f64>>,
    old_velocity: Array1<f64>,
    velocity: Array1<f64>,
    channel: (Sender<NewVelAdjustment>, Receiver<NewVelAdjustment>),
    calibration_stage: CalibrationStage,
    config: PicarConfig,
    picar: Picar,
    _after_update: bool,
}

impl OrcaCar {
    /// Create a new instance of an OrcaCar
    pub async fn new(
        position: Option<Array1<f64>>,
        target: Option<Array1<f64>>,
    ) -> Result<Self, Box<dyn Error>> {
        debug!("OrcaCar::new({:?}, {:?})", position, target);
        //
        debug!("Successfully created OrcaCar!");
        Ok(Self {
            position,
            target,
            old_velocity: arr1(&[0.0, 0.0]),
            velocity: arr1(&[0.0, 0.0]),
            channel: async_channel::unbounded::<NewVelAdjustment>(),
            calibration_stage: CalibrationStage::No,
            config: PicarConfig::default(),
            picar: Picar::new().await,
            _after_update: false,
        })
    }

    /// Update the information about this car.
    /// Note: This should be called each time before calling OrcaCar::tick!
    pub fn update(&mut self, position: Array1<f64>, target: Array1<f64>) {
        debug!("OrcaCar::update({}, {:?})", position, target);
        if self._after_update {
            warn!("Calling OrcaCar::update() twice without calling OrcaCar::tick()!");
        }
        self.old_velocity = self.velocity.clone();
        debug!("self.old_velocity = {:?}", self.old_velocity);
        self.velocity = &position
            - match self.position.clone() {
                None => arr1(&[0.0, 0.0]),
                Some(pos) => pos,
            };
        debug!("self.velocity = {:?}", self.velocity);
        self.position = Some(position);
        debug!("self.position = {:?}", self.position);
        self.target = Some(target);
        debug!("self.target = {:?}", self.target);
        self._after_update = true;
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
        radius: f64,
        confidence: f64,
    ) -> Result<(Option<Array1<f64>>, bool), Box<dyn Error>> {
        debug!("OrcaCar::tick({:?}, {}, {})", others, radius, confidence);
        if !self._after_update {
            warn!("Calling OrcaCar::tick() without calling OrcaCar::update() before!");
        }
        self._after_update = false;
        if let CalibrationStage::Done = self.calibration_stage {
            self.orca_tick(others, radius, confidence).await
        } else {
            self.calibration_tick().await?;
            Ok((None, false))
        }
    }

    /// Perform a tick of the initial calibration of this car.
    /// This includes calculation of max speed, amount of turn in direction left and right.
    async fn calibration_tick(&mut self) -> Result<Option<Array1<f64>>, Box<dyn Error>> {
        debug!("OrcaCar::calibration_tick()");
        match self.calibration_stage.clone() {
            CalibrationStage::No => {
                self.start_max_speed().await?;
                self.calibration_stage = CalibrationStage::DuringMaxSpeed;
            }
            CalibrationStage::DuringMaxSpeed => {
                self.calc_max_speed();
                self.start_right_turn().await?;
                self.calibration_stage = CalibrationStage::DuringRightTurn;
            }
            CalibrationStage::DuringRightTurn => {
                self.calc_right_turn();
                self.start_left_turn().await?;
                self.calibration_stage = CalibrationStage::DuringLeftTurn;
            }
            CalibrationStage::DuringLeftTurn => {
                self.calc_left_turn();
                self.finish_calibration();
                self.calibration_stage = CalibrationStage::Done;
            }
            CalibrationStage::Done => unreachable!(),
        }
        Ok(None)
    }

    /// This thing should be called at the start of the calibration.
    /// Drive for a certain time and then stop.
    async fn start_max_speed(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("OrcaCar::start_max_speed()");
        self.picar.drive().await?;
        thread::sleep(Duration::from_millis(DRIVE_DURATION));
        self.picar.stop().await
    }

    /// Calculate the max speed depending on the new position
    fn calc_max_speed(&mut self) {
        debug!("OrcaCar::calc_max_speed()");
        self.config.max_speed = norm(&self.velocity);
        debug!("self.config.max_speed = {}", self.config.max_speed);
    }

    /// Try to calculate the amount of "right turn" (i.e., how much does the car turn right).
    /// We first turn, then drive, then stop.
    async fn start_right_turn(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("OrcaCar::start_right_turn()");
        self.picar.turn_right().await?;
        thread::sleep(Duration::from_millis(TURN_DURATION));
        self.picar.drive().await?;
        thread::sleep(Duration::from_millis(DRIVE_DURATION));
        self.picar.stop().await
    }

    /// Calculate the actual amount of "turn right" depending on the new velocity.
    /// This works by calculating the angle between the last velocity and the new velocity.
    fn calc_right_turn(&mut self) {
        debug!("OrcaCar::calc_right_turn()");
        self.config.right_turn = signed_angle_between(&self.old_velocity, &self.velocity);
        debug!("self.config.right_turn = {}", self.config.right_turn);
    }

    /// Try to calculate the amount of "left turn" (i.e., how much does the car turn left).
    /// We first turn, then drive, then stop.
    async fn start_left_turn(&mut self) -> Result<(), Box<dyn Error>> {
        debug!("OrcaCar::start_left_turn()");
        self.picar.turn_left().await?;
        thread::sleep(Duration::from_millis(TURN_DURATION));
        self.picar.drive().await?;
        thread::sleep(Duration::from_millis(DRIVE_DURATION));
        self.picar.stop().await
    }

    /// Calculate the actual amount of "turn left" depending on the new velocity.
    /// This works by calculating the angle between the last velocity and the new velocity.
    fn calc_left_turn(&mut self) {
        debug!("OrcaCar::calc_left_turn()");
        self.config.left_turn = signed_angle_between(&self.old_velocity, &self.velocity);
        debug!("self.config.left_turn = {}", self.config.left_turn);
    }

    /// Finish the calibration and start the car with the actual "logic".
    fn finish_calibration(&mut self) {
        debug!("OrcaCar::finish_calibration()");
        let (_, receiver) = self.channel.clone();

        let config = self.config;
        let mut picar = self.picar.clone();
        tokio::spawn(async move {
            loop {
                // try to fetch new direction
                debug!("Trying to receive from sender...");
                match timeout(Duration::from_millis(TIMEOUT), receiver.recv()).await {
                    Ok(res) => {
                        debug!("Received {:?}", res);
                        match res {
                            Ok((old_vel, new_vel)) => {
                                // check, if we have to turn right or left
                                let angle = signed_angle_between(&old_vel, &new_vel);
                                if angle.is_sign_positive() {
                                    if let Err(e) = picar.turn_right().await {
                                        error!("Error during picar.turn_right() ({})", e);
                                    }
                                    thread::sleep(Duration::from_millis(
                                        ((angle / config.right_turn) * TURN_DURATION as f64 / 2.0)
                                            as u64,
                                    ));
                                } else {
                                    if let Err(e) = picar.turn_left().await {
                                        error!("Error during picar.turn_left() ({})", e);
                                    }
                                    thread::sleep(Duration::from_millis(
                                        ((angle / config.left_turn) * TURN_DURATION as f64 / 2.0)
                                            as u64,
                                    ));
                                }
                                // and driiiiiiivvvvveee
                                if let Err(e) = picar.drive().await {
                                    error!("Error during picar.drive() ({})", e);
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
                        if let Err(e) = picar.stop().await {
                            error!("Error during picar.stop() ({})", e);
                        }
                    }
                }
            }
        });
    }

    async fn orca_tick(
        &mut self,
        mut others: Vec<Participant>,
        radius: f64,
        confidence: f64,
    ) -> Result<(Option<Array1<f64>>, bool), Box<dyn Error>> {
        debug!(
            "OrcaCar::orca_tick({:?}, {}, {})",
            others, radius, confidence
        );

        // TODO: Do we actually have them all the time?
        if dist(
            &self.position.clone().unwrap(),
            &self.target.clone().unwrap(),
        ) < DISTANCE_THRESHOLD
        {
            self.picar.stop().await?;
            return Ok((None, true));
        }

        let old_velocity = self.velocity.clone();
        let mut new_vel = arr1(&[0.0, 0.0]);

        if let Some(position) = self.position.clone() {
            if let Some(target) = self.target.clone() {
                let mut we =
                    Participant::new(position.clone(), self.velocity.clone(), radius, confidence)
                        .with_inner_state(self.config.max_speed, target);
                we.update_position(&position);
                new_vel = we.orca(&mut others, &[], 10.0);
            }
        }

        let (sender, _) = self.channel.clone();

        debug!("Sending ({}, {})", old_velocity, new_vel);
        sender.send((old_velocity, new_vel.clone())).await?;
        Ok((Some(new_vel), false))
    }
}
