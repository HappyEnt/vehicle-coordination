use std::thread;
use std::time::Duration;

use async_channel::{Receiver, Sender};
use rppal::pwm::{Channel, Polarity, Pwm};
use tonic::{transport::Server, Request, Response, Status};

use picar::picar_server::{Picar, PicarServer};
use picar::{SetSpeedRequest, SetSpeedResponse};

pub mod picar {
    tonic::include_proto!("picar");
}

const HERTZ: f64 = 50.0;
const FULL_DUTY_LENGTH: f64 = 1000.0 / HERTZ;
const MIN_DUTY_RATIO: f64 = 1.0 / FULL_DUTY_LENGTH;
const MAX_DUTY_RATIO: f64 = 2.0 / FULL_DUTY_LENGTH;

#[derive(Debug)]
pub struct PicarService {
    channel: Sender<(f64, f64)>,
}

fn calc_duty_ratio(val: f64) -> f64 {
    // quick maffs
    MIN_DUTY_RATIO + (MAX_DUTY_RATIO - MIN_DUTY_RATIO) * (0.5 * (val + 1.0))
}

#[tonic::async_trait]
impl Picar for PicarService {
    async fn set_speed(
        &self,
        request: Request<SetSpeedRequest>,
    ) -> Result<Response<SetSpeedResponse>, Status> {
        let req = request.into_inner();
        println!("inner: {:?}", req);

        let left = req.left as f64;
        let right = req.right as f64;

        if left.abs() > 1.0 || right.abs() > 1.0 {
            return Ok(Response::new(SetSpeedResponse {
                success: false,
                message: format!("{} and {} have to be between -1 and 1!", left, right),
            }));
        }

        let duty_ratio_l = calc_duty_ratio(left);
        let duty_ratio_r = calc_duty_ratio(right);

        // TODO: Sleep for a certain amount of time
        // ? This has to be send via the request.
        if self
            .channel
            .send((duty_ratio_l, duty_ratio_r))
            .await
            .is_err()
        {
            eprintln!("Could not set PWM duty cycle");
            Err(Status::internal("Could not set PWM duty cycle"))
        } else {
            let reply = SetSpeedResponse {
                message: "Speed set successfully".to_owned(),
                success: true,
            };
            Ok(Response::new(reply))
        }
    }
}

impl PicarService {
    fn start(&self, receiver: Receiver<(f64, f64)>) {
        thread::spawn(move || {
            let pwm_left =
                Pwm::with_frequency(Channel::Pwm0, HERTZ, 0.075, Polarity::Normal, true).unwrap();
            let pwm_right =
                Pwm::with_frequency(Channel::Pwm1, HERTZ, 0.075, Polarity::Normal, true).unwrap();

            let mut current_l = 0.075;
            let mut current_r = 0.075;

            let mut desired_l = current_l;
            let mut desired_r = current_r;

            loop {
                match receiver.try_recv() {
                    Err(_) => (),
                    Ok((left, right)) => {
                        desired_l = left;
                        desired_r = right;
                    }
                };

                let left_diff = desired_l - current_l;
                let left_change = 0.05_f64.min(left_diff.abs()) * left_diff.signum();
                current_l += left_change;

                let right_diff = desired_r - current_r;
                let right_change = 0.05_f64.min(right_diff.abs()) * right_diff.signum();
                current_r += right_change;

                if let Err(_) = pwm_left.set_duty_cycle(current_l) {
                    eprintln!("Could not set PWM duty cycle");
                } else if let Err(_) = pwm_right.set_duty_cycle(current_r) {
                    eprintln!("Could not set PWM duty cycle");
                }
                thread::sleep(Duration::from_millis(100));
            }
        });
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr = "0.0.0.0:50051".parse()?;

    let (sender, receiver) = async_channel::unbounded::<(f64, f64)>();

    let picar_service = PicarService { channel: sender };

    picar_service.start(receiver);

    Server::builder()
        .add_service(PicarServer::new(picar_service))
        .serve(addr)
        .await?;

    Ok(())
}
