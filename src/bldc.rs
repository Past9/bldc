use core::time::Duration;

use crate::{
  current_controller::CurrentController,
  drv_8305::ReadCommand,
  drv_8305::WarningFlag,
  drv_8305::Warnings,
  drv_8305::{Command, Drv8305},
  runner::Program,
};
use cortex_m_semihosting::hprintln;
use stm32f303_api::{
  clocks::{
    AhbPrescalerValue, Apb1PrescalerValue, Apb2PrescalerValue, ClockConfig, McoSourceMuxInput,
    PllMulValue, PllSourceMuxInput, SystemClockMuxInput,
  },
  gpio::gpio_b::GpioB,
  gpio::gpio_e::GpioE,
  Result, System,
};

pub struct Bldc {
  system: System,
  gpio_b: GpioB,
  gpio_e: GpioE,
  driver: Drv8305,
  current_controller: CurrentController,
  angle: f32,
  speed: f32,
  power: f32,
}
impl Bldc {
  pub fn new() -> Result<Bldc> {
    let mut clock_cfg = ClockConfig::with_freqs(0, 0);

    clock_cfg.set_pll_source_mux_input(PllSourceMuxInput::Hsi);
    clock_cfg.set_pll_mul_factor(PllMulValue::Mul16);
    clock_cfg.set_system_clock_mux_input(SystemClockMuxInput::Pll);
    clock_cfg.set_ahb_prescaler_divisor(AhbPrescalerValue::NoDiv);
    clock_cfg.set_apb1_prescaler_divisor(Apb1PrescalerValue::Div8);
    clock_cfg.set_apb2_prescaler_divisor(Apb2PrescalerValue::Div4);
    clock_cfg.set_mco_source_mux_input(McoSourceMuxInput::Hsi);

    let mut system = System::with_clocks(clock_cfg)?;
    let mut gpio_b = system.activate_gpio_b()?;
    let mut gpio_e = system.activate_gpio_e()?;

    let mut driver = Drv8305::new(&mut system, &mut gpio_b)?;
    driver.start();

    let mut current_controller = CurrentController::new(
      &mut system,
      &mut gpio_e,
      20000f32,
      Duration::from_nanos(500),
    )?;

    current_controller.set_angle(0f32, 0.2f32)?;

    current_controller.start();

    Ok(Self {
      system,
      gpio_b,
      gpio_e,
      driver,
      current_controller,
      angle: 0f32,
      speed: 0f32,
      power: 0.2f32,
    })
  }
}
impl Program for Bldc {
  fn step(&mut self) -> Result<()> {
    let warnings = self.driver.read_warnings()?;

    if warnings.ok() {
      self.driver.enable_gate();

      self.speed += 0.00001;

      if self.speed > 0.05 {
        self.speed = 0.05;
      }

      self.angle += self.speed;

      self.current_controller.set_angle(self.angle, self.power)?;
    } else {
      self.driver.disable_gate();
      self.speed = 0f32;

      if warnings.has(WarningFlag::Overtemp) {
        hprintln!("Overtemp").unwrap();
      }
      if warnings.has(WarningFlag::TempOver135C) {
        hprintln!("Temp over 135 C").unwrap();
      }
      if warnings.has(WarningFlag::TempOver125C) {
        hprintln!("Temp over 125 C").unwrap();
      }
      if warnings.has(WarningFlag::TempOver105C) {
        hprintln!("Temp over 105 C").unwrap();
      }
      if warnings.has(WarningFlag::ChargePumpUndervolt) {
        hprintln!("Charge pump undervolt").unwrap();
      }
      if warnings.has(WarningFlag::VdsOvercurrent) {
        hprintln!("VDS overcurrent").unwrap();
      }
      if warnings.has(WarningFlag::PvddOvervolt) {
        hprintln!("PVDD overvolt").unwrap();
      }
      if warnings.has(WarningFlag::PvddUndervolt) {
        hprintln!("PVDD undervolt").unwrap();
      }
      if warnings.has(WarningFlag::TempOver175C) {
        hprintln!("Temp over 175 C").unwrap();
      }
      if warnings.has(WarningFlag::Fault) {
        hprintln!("FAULT").unwrap();
      }
    }

    Ok(())
  }

  fn shutdown(self) -> Result<()> {
    Ok(())
  }

  fn should_continue(&mut self) -> Result<bool> {
    Ok(true)
  }
}
