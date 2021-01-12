#![feature(core_intrinsics)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

mod bldc;
mod current_controller;
mod drv_8305;
mod runner;

use bldc::Bldc;
use cortex_m_rt::entry;

#[entry]
#[no_mangle]
fn main() -> ! {
  runner::run(Bldc::new());
}
