# SmolPhone

The SmolPhone stack is divided into multiple subprojects organized in a Cargo workspace.

## Butler
The `butler` subproject is meant to run on a Rapsberry Pi Pico and contains the logic
to run a simple program on the MCU, switching on the performance core as needed
before acting as a USB hub to access the rest of the phone's peripherals (keyboard, screen, etc).

### Prerequisites
```sh
# Target for the Rapsberry Pi Pico
rustup target add thumbv6m-none-eabi
# Used to flash on the board
cargo install elf2uf2-rs --locked
```

To compile it, simply use `cargo run -p butler` while a Rapsberry Pi Pico is mounted
in BOOTSEL mode to your computer.

## Legacy
The legacy code is meant to run on the performance core. It it meant to run on a
Linux operating system.

UDEV rules are given in the form of `99-pico.rules`. This file should be placed in
`/etc/udev/rules.d/` on the performance core, then run `udevadm control --reload-rules` 
to apply said rules.

The legacy program can be run like any other Rust program. If you decide to cross-compile
it, here are the prerequisites:
```sh
# Target for ARM boards
rustup target add armv7-unknown-linux-gnueabihf
sudo apt install gcc-arm-linux-gnueabihf
```

## Local offloading
TODO
