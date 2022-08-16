use log::error;
use std::{fmt::Debug, str::FromStr};

/// Try to parse the provided value to the desired type.
pub fn parse_cmd_arg<T: FromStr>(arg: &String) -> T
where
    <T as FromStr>::Err: Debug,
{
    let parsed_arg = arg.parse::<T>();
    match parsed_arg {
        Ok(val) => val,
        Err(err) => {
            error!("Error while parsing '{}': {:?}", arg, err);
            std::process::exit(-1)
        }
    }
}
