# Picar wheels

This repo contains the code for working with the wheels of our Raspis.

## Overview

-   `src/daemon.rs` contains the code for the daemon running in the background, which is connected to the PWM channels
-   `src/client.rs` contains the code for the actual executable to interact with the daemon
-   `proto/picar.proto` contains the protobuf definitions for communication via RPC with the daemon

## Communication

To communicate with the daemon, possible clients can use gRPC. As a protobuf schema, we defined:

```proto
syntax = "proto3";

package picar;

service Picar {
    rpc SetSpeed (SetSpeedRequest) returns (SetSpeedResponse);
}

message SetSpeedRequest {
    double left = 1;
    double right = 2;
}

message SetSpeedResponse {
    bool success = 1;
    string message = 2;
}
```

The daemon expects a message with fields `left` and `right` to lie between **-1** and **1**. **-1** mean full power backwards, whereas **1** means full power forwards.

## Compilation

To compile this project via cargo, simply run

```sh
cargo build --bins --release
```

This builds binaries for both the client and the daemon. Since the project, by default, uses `armv7-unknown-linux-gnueabihf` as the target for the compilation, you need several things setup on your machine.

### Rustup Target

You need to run `rustup target add armv7-unknown-linux-gnueabihf` to install the compilation target for ARM64.

### ARM 7 Toolchain

Cargo expects you to have the native toolchain installed on you system to create binaries (libraries would work without this toolchain). To correctly setup you environment, you need to have these environment variables defined:

```sh
CC_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-gcc
CXX_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-g++
AR_armv7_unknown_linux_gnueabihf=armv7-unknown-linux-gnueabihf-ar
CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=armv7-unknown-linux-gnueabihf-gcc
```

You can obtain these binaries from `https://github.com/messense/homebrew-macos-cross-toolchains`.
