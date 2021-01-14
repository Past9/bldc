use core::time::Duration;

use crate::modes::{calibration::CalibrationMode, demo::DemoMode, recovery::RecoveryMode};
use crate::{
  drv_8305::Drv8305, drv_8305::WarningFlag, magnet_controller::MagnetController,
  position_sensor::PositionSensor, runner::Program,
};
use core::fmt::Write;
use stm32f303_api::{
  clocks::{
    AhbPrescalerValue, Apb1PrescalerValue, Apb2PrescalerValue, ClockConfig, McoSourceMuxInput,
    PllMulValue, PllSourceMuxInput, SystemClockMuxInput,
  },
  gpio::gpio_a::GpioA,
  gpio::gpio_b::GpioB,
  gpio::gpio_e::GpioE,
  Result, System,
};

pub enum Mode {
  Start,
  Calibrate(CalibrationMode),
  Demo(DemoMode),
}

pub struct Bldc {
  recovery_mode: Option<RecoveryMode>,
  num_magnet_pairs: u32,
  mode: Mode,
  system: System,
  gpio_a: GpioA,
  gpio_b: GpioB,
  gpio_e: GpioE,
  drv_8305: Drv8305,
  magnet_controller: MagnetController,
  position_sensor: PositionSensor,
}
impl Bldc {
  pub fn new(num_magnet_pairs: u32) -> Result<Bldc> {
    let mut clock_cfg = ClockConfig::with_freqs(0, 0);

    clock_cfg.set_pll_source_mux_input(PllSourceMuxInput::Hsi);
    clock_cfg.set_pll_mul_factor(PllMulValue::Mul16);
    clock_cfg.set_system_clock_mux_input(SystemClockMuxInput::Pll);
    clock_cfg.set_ahb_prescaler_divisor(AhbPrescalerValue::NoDiv);
    clock_cfg.set_apb1_prescaler_divisor(Apb1PrescalerValue::Div8);
    clock_cfg.set_apb2_prescaler_divisor(Apb2PrescalerValue::Div4);
    clock_cfg.set_mco_source_mux_input(McoSourceMuxInput::Hsi);

    let mut system = System::with_clocks(clock_cfg)?;
    let mut gpio_a = system.activate_gpio_a()?;
    let mut gpio_b = system.activate_gpio_b()?;
    let mut gpio_e = system.activate_gpio_e()?;

    let mut drv_8305 = Drv8305::new(&mut system, &mut gpio_b)?;
    drv_8305.start();

    let mut current_controller = MagnetController::new(
      &mut system,
      &mut gpio_e,
      20000f32,
      Duration::from_nanos(500),
    )?;

    current_controller.set_phase_angle_and_power(0f32, 0f32)?;
    current_controller.start();

    let mut position_sensor = PositionSensor::new(num_magnet_pairs, &mut system, &mut gpio_a)?;
    position_sensor.start();

    Ok(Self {
      recovery_mode: None,
      num_magnet_pairs,
      mode: Mode::Start,
      system,
      gpio_a,
      gpio_b,
      gpio_e,
      drv_8305,
      magnet_controller: current_controller,
      position_sensor,
    })
  }

  fn handle_drv_8305_errors(&mut self) -> Result<()> {
    let warnings = self.drv_8305.read_warnings()?;

    if warnings.ok() {
      self.recovery_mode = None;
    } else {
      self.drv_8305.disable_gate();
      self.recovery_mode = Some(RecoveryMode::new());

      if warnings.has(WarningFlag::Overtemp) {
        println!("Overtemp").ok();
      }
      if warnings.has(WarningFlag::TempOver135C) {
        println!("Temp over 135 C").ok();
      }
      if warnings.has(WarningFlag::TempOver125C) {
        println!("Temp over 125 C").ok();
      }
      if warnings.has(WarningFlag::TempOver105C) {
        println!("Temp over 105 C").ok();
      }
      if warnings.has(WarningFlag::ChargePumpUndervolt) {
        println!("Charge pump undervolt").ok();
      }
      if warnings.has(WarningFlag::VdsOvercurrent) {
        println!("VDS overcurrent").ok();
      }
      if warnings.has(WarningFlag::PvddOvervolt) {
        println!("PVDD overvolt").ok();
      }
      if warnings.has(WarningFlag::PvddUndervolt) {
        println!("PVDD undervolt").ok();
      }
      if warnings.has(WarningFlag::TempOver175C) {
        println!("Temp over 175 C").ok();
      }
      if warnings.has(WarningFlag::Fault) {
        println!("FAULT").ok();
      }
    }

    Ok(())
  }
}
impl<'a> Program for Bldc {
  fn step(&mut self) -> Result<()> {
    self.handle_drv_8305_errors()?;
    match &mut self.recovery_mode {
      Some(recovery_mode) => recovery_mode.step(
        &mut self.drv_8305,
        &mut self.magnet_controller,
        &mut self.position_sensor,
      ),
      None => match &mut self.mode {
        Mode::Start => {
          self.mode = Mode::Calibrate(CalibrationMode::new());
          Ok(())
        }
        Mode::Calibrate(calibration_mode) => {
          calibration_mode.step(
            &mut self.drv_8305,
            &mut self.magnet_controller,
            &mut self.position_sensor,
          )?;
          if calibration_mode.is_done() {
            self.mode = Mode::Demo(DemoMode::new(
              &mut self.drv_8305,
              &mut self.magnet_controller,
            )?);
          }
          Ok(())
        }
        Mode::Demo(demo_mode) => {
          demo_mode.step(
            &mut self.drv_8305,
            &mut self.magnet_controller,
            &mut self.position_sensor,
          )?;
          Ok(())
        }
      },
    }
  }

  fn safemode(&mut self) {
    // Brake mode
    self.magnet_controller.set_power_scale(0f32).ok();
    self.drv_8305.disable_gate();
  }

  fn shutdown(mut self) -> Result<()> {
    self
      .magnet_controller
      .return_hardware(&mut self.system, &mut self.gpio_e)?;

    self
      .drv_8305
      .return_hardware(&mut self.system, &mut self.gpio_b)?;

    self
      .position_sensor
      .return_hardware(&mut self.system, &mut self.gpio_a)?;

    Ok(())
  }

  fn should_continue(&mut self) -> Result<bool> {
    Ok(true)
  }
}
