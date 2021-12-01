#![no_std]
#![cfg_attr(test, no_main)]

use stellantis_can_adapter as _; // memory layout + panic handler

#[defmt_test::tests]
mod tests {}
