use picar::picar_client::PicarClient;
use picar::SetSpeedRequest;

pub mod picar {
    tonic::include_proto!("picar");
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let mut left = parse_cmd_arg(&args[1]);
    let mut right = parse_cmd_arg(&args[2]) * -1.0;

    // clamp values to lie between -1 and 1
    left = left.clamp(-1.0, 1.0);
    right = right.clamp(-1.0, 1.0);

    let mut client = PicarClient::connect("http://0.0.0.0:50051").await?;

    let request = tonic::Request::new(SetSpeedRequest { left, right });

    let response = client.set_speed(request).await?;
    let res = response.into_inner();

    // println!("Response = {:?}", response);

    if !res.success {
        eprintln!("{}", res.message);
        std::process::exit(-1);
    }

    Ok(())
}

fn parse_cmd_arg(arg: &String) -> f64 {
    let parsed_arg = arg.parse::<f64>();
    match parsed_arg {
        Ok(val) => val,
        Err(err) => {
            eprintln!("Error while parsng '{}': {}", arg, err);
            std::process::exit(-1)
        }
    }
}
