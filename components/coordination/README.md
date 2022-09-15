# Picar Coordination

This repo contains the code for the coordination of our Picars. Furthermore, it handles initial "calibration" of a car, i.e., determining the rotation, max speed, etc..

## Overview 

- `src/bin` this folder holds both binaries of this crate (one for the actual coordination and one as the camera interface)
- `src/*` these files contain different utility structs and functionality, depending on the name of the file 

## Compilation

To compile both binaries for your local system, simply run 

```shell
make native
```

To compile them for the Pis, run 

```shell
make pi
```

**Note:** You need to have the respective rustup target and ARM 7 toolchains installed on your system!

### Rustup target

You need to run `rustup target add armv7-unknown-linux-gnueabihf` to install the compilation target for ARM64.

### ARM 7 toolchain

Cargo expects you to have the native toolchain installed on you system to create binaries (libraries would work without this toolchain). To correctly setup you environment, you need to have these environment variables defined:

```sh
CC_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-gcc
CXX_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-g++
AR_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-ar
CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=armv7-unknown-linux-gnueabihf-gcc
```

You can obtain these binaries from `https://github.com/messense/homebrew-macos-cross-toolchains`.
