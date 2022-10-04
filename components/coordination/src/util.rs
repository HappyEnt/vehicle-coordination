//! Some (maybe needed) utility functions.

use log::error;
use orca_rs::{ndarray::arr1, obstacle::Obstacle};
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

pub fn generate_bounding_obstacles(width: f64, height: f64) -> [Obstacle; 4] {
    [
        Obstacle {
            start: arr1(&[0.0, 0.0]),
            end: arr1(&[0.0, height]),
            radius: 0.01,
        },
        Obstacle {
            start: arr1(&[0.0, height]),
            end: arr1(&[width, height]),
            radius: 0.01,
        },
        Obstacle {
            start: arr1(&[width, height]),
            end: arr1(&[width, 0.0]),
            radius: 0.01,
        },
        Obstacle {
            start: arr1(&[width, 0.0]),
            end: arr1(&[0.0, 0.0]),
            radius: 0.01,
        },
    ]
}
