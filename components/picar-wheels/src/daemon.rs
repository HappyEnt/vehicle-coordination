use rppal::pwm::{Channel, Polarity, Pwm};
use tonic::{transport::Server, Request, Response, Status};

use picar::picar_server::{Picar, PicarServer};
use picar::{SetSpeedRequest, SetSpeedResponse};

pub mod picar {
    tonic::include_proto!("picar");
}

const HERTZ: f64 = 50.0;

#[derive(Debug)]
pub struct PicarService {
    pwm_left: Pwm,
    pwm_right: Pwm,
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

        // quick maffs
        let full_duty_length = 1000.0 / HERTZ;
        let min_duty_ratio = 1.0 / full_duty_length;
        let max_duty_ratio = 2.0 / full_duty_length;

        let duty_ratio_l =
            min_duty_ratio + (max_duty_ratio - min_duty_ratio) * (0.5 * (left + 1.0));
        let duty_ratio_r =
            min_duty_ratio + (max_duty_ratio - min_duty_ratio) * (0.5 * (right + 1.0));

        // try to set pwm speeds
        if let Err(_) = self.pwm_left.set_duty_cycle(duty_ratio_l) {
            eprintln!("Could not set PWM duty cycle");
            Err(Status::internal("Could not set PWM duty cycle"))
        } else if let Err(_) = self.pwm_right.set_duty_cycle(duty_ratio_r) {
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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr = "[::1]:50051".parse()?;
    let picar_service = PicarService {
        pwm_left: Pwm::with_frequency(Channel::Pwm0, HERTZ, 0.075, Polarity::Normal, true)?,
        pwm_right: Pwm::with_frequency(Channel::Pwm1, HERTZ, 0.075, Polarity::Normal, true)?,
    };

    Server::builder()
        .add_service(PicarServer::new(picar_service))
        .serve(addr)
        .await?;

    Ok(())
}
