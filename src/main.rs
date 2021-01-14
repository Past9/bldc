#![feature(core_intrinsics)]
#![no_std]
#![no_main]

#[allow(unused_macros)]
macro_rules! print {
  ($($arg:tt)*) => ({
    match cortex_m_semihosting::hio::hstdout() {
      Ok(ref mut fd) => match write!(fd, $($arg)*) {
        Ok(()) => Ok(()),
        Err(_) => Err(stm32f303_api::Error::new("Could not write to host IO")),
      },
      Err(()) => Err(stm32f303_api::Error::new("Could not get host IO")),
    }
  })
}

#[allow(unused_macros)]
macro_rules! println {
  ($($arg:tt)*) => ({
    match print!($($arg)*) {
      Ok(()) => print!("\n"),
      Err(e) => Err(e)
    }
  })
}

const NUM_MAGNET_PAIRS: u32 = 20;

extern crate panic_semihosting;

mod bldc;
mod drv_8305;
mod magnet_controller;
mod math;
mod modes;
mod position_sensor;
mod runner;

use bldc::Bldc;
use cortex_m_rt::entry;

#[entry]
#[no_mangle]
fn main() -> ! {
  runner::run(Bldc::new(NUM_MAGNET_PAIRS));
}
