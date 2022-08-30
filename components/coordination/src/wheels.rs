use log::{debug, error};
use std::error::Error;
use tonic::transport::Channel;

use self::picar::picar_client::PicarClient;
use self::picar::SetSpeedRequest;

mod picar {
    tonic::include_proto!("picar");
}

/// Abstraction around the wheels of each car.
#[derive(Clone)]
pub struct Wheels {
    client: PicarClient<Channel>,
}

impl Wheels {
    /// Create a new instance and connect to localhost daemon
    pub async fn new() -> Result<Self, Box<dyn Error>> {
        debug!("Wheels::new()");
        Self::from_address("http://0.0.0.0:50051").await
    }

    /// Create a new instance and connecto to daemon at given address.
    pub async fn from_address(address: &str) -> Result<Self, Box<dyn Error>> {
        debug!("Wheels::from_adress({})", address);
        Ok(Wheels {
            client: PicarClient::connect(address.to_owned()).await?,
        })
    }

    /// Set the speed of both wheels of the car.
    pub async fn set_speed(&mut self, left: f64, right: f64) -> Result<(), Box<dyn Error>> {
        debug!("Wheels::set_speed({}, {})", left, right);
        let left = left.clamp(-1.0, 1.0);
        let right = right.clamp(-1.0, 1.0) * -1.0;

        let request = tonic::Request::new(SetSpeedRequest { left, right });
        if let Err(e) = self.client.set_speed(request).await {
            error!("Error setting speed: {}", e);
        }
        debug!("Successfully send speed request");

        Ok(())
    }
}
