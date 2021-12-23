# Stellantis CAN Adapter

The Stellantis CAN Adapter is a `#![no_std]` Rust implementation of the PSA
CAN2004/CAN2010 confort bridge [project][can-brigde-repo].

[can-brigde-repo]: https://github.com/ludwig-v/arduino-psa-comfort-can-adapter

## Why?
Switching from CAN2004 to CAN2010, PSA moved some functions from the telematic
unit to the BSI :
* Time clock
* Language settings
And also units and frames ids changed on CAN bus, which makes barely possible
to retrofit a newer telematic unit (such as NAC) to a CAN2004 car.

## Goals
This project is in the starting phase, the goals are intentionally ridiculous.
The first task is to implement the time clock following the requirements.

## Requirements
The CAN adapter should implement:
* A time clock
    * Should be settable
    * Should send periodically the time on CAN bus.

## Target Devices
This project has been tested on a STM32F105RC MCU.
Specifically, it was written to work with the "CAN filter" boards
described in [this article][can-filter-article], and it makes the following
assumptions:

* Both CAN1 and CAN2 are connected to the same CAN bus, with no interfering
  devices on the bus.
* CAN1 is connected to pins PA11 and PA12.
* CAN2 is connected to pins PB5 and PB6 (a non-default remapping).

[`defmt-test`]: https://crates.io/crates/defmt-test
[can-filter-article]: https://dangerouspayload.com/2020/03/10/hacking-a-mileage-manipulator-can-bus-filter-device/

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
