//! # Picar Coordination
//! This crate contains all modules needed for the coordination of our picars.  
//!
//! ## Overview
//!
//! - `src/bin` this folder holds both binaries of this crate (one for the actual coordination and one as the camera interface)
//! - `src/*` these files contain different utility structs and functionality, depending on the name of the file
//!
//! ### Binaries
//!
//! #### Picar Coordination
//!
//! To start the actual coordination, you can start the executable `picar-coordination`. The program will try to connect to a locally running `picar-daemon` (so start it first!) and then waits for incomming connections for receiving information about other participants.
//!
//! #### Camera Interface
//!
//! In order to provide information about other participants via the camera server, this crate provides the `camera-interface` binary. This program is responsible for connecting to a *running* camera server and to a *locally running* `picar-coordination`. If one of them is not available, it will panic!
//!
//! To provide reasonable data, the program needs to be called with special arguments:
//!
//! ```shell
//! camera-interface <ID>
//! ```
//! `<ID>` is the ID of the respective car.
//!
//! Currently, the binary tries to connect to a camera server reachable under `192.168.87.78:8081` _in the same network_ (adjustable in the code) and lets each car drive from one corner to the other (also adjustable in the code).  
//!
//! ## Compilation
//!
//! To compile both binaries for your local system, simply run
//!
//! ```shell
//! make native
//! ```
//!
//! To compile them for the Pis, run
//!
//! ```shell
//! make pi
//! ```
//!
//! **Note:** You need to have the respective rustup target and ARM 7 toolchains installed on your system!
//!
//! ### Rustup target
//!
//! You need to run `rustup target add armv7-unknown-linux-gnueabihf` to install the compilation target for ARM64.
//!
//! ### ARM 7 toolchain
//!
//! Cargo expects you to have the native toolchain installed on you system to create binaries (libraries would work without this toolchain). To correctly setup you environment, you need to have these environment variables defined:
//!
//! ```sh
//! CC_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-gcc
//! CXX_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-g++
//! AR_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-ar
//! CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=armv7-unknown-linux-gnueabihf-gcc
//! ```
//!
//! You can obtain these binaries from `https://github.com/messense/homebrew-macos-cross-toolchains`.

pub mod interface;
pub mod math;
pub mod orca_car;
pub mod util;
pub mod wheels;
