use core::fmt::Write;
use core::time::Duration;
use stm32f303_api::{
  gpio::gpio_e::{
    GpioE, Pe10AltFunc, Pe10Tim1Ch2n, Pe11AltFunc, Pe11Tim1Ch2, Pe12AltFunc, Pe12Tim1Ch3n,
    Pe13AltFunc, Pe13Tim1Ch3, Pe8AltFunc, Pe8Tim1Ch1n, Pe9AltFunc, Pe9Tim1Ch1,
  },
  gpio::OutputSpeed,
  gpio::OutputType,
  gpio::PullDirection,
  timer::tim1::Ch1CompareMode,
  timer::OutputChannel,
  timer::{
    tim1::{Ch1Output, Ch2CompareMode, Ch2Output, Ch3CompareMode, Ch3Output, Tim1},
    Timer,
  },
  Result, System,
};

use crate::math::norm_rads;

const PI: f32 = 3.14159;
const PI2: f32 = PI * 2f32;
const PI2_3: f32 = PI2 / 3f32;
const PI4_3: f32 = PI2_3 * 2f32;

pub struct MagnetController {
  timer: Tim1,

  ch_u_pwm: Ch1Output,
  ch_v_pwm: Ch2Output,
  ch_w_pwm: Ch3Output,

  ch_u_pin: Pe9AltFunc<Pe9Tim1Ch1>,
  ch_v_pin: Pe11AltFunc<Pe11Tim1Ch2>,
  ch_w_pin: Pe13AltFunc<Pe13Tim1Ch3>,

  ch_un_pin: Pe8AltFunc<Pe8Tim1Ch1n>,
  ch_vn_pin: Pe10AltFunc<Pe10Tim1Ch2n>,
  ch_wn_pin: Pe12AltFunc<Pe12Tim1Ch3n>,

  phase_angle: f32,
  power_scale: f32,
}
impl MagnetController {
  pub fn new(
    system: &mut System,
    gpio_e: &mut GpioE,
    pwm_freq: f32,
    deadtime: Duration,
  ) -> Result<Self> {
    let mut timer = system.activate_tim1()?;
    timer.config_as_pwm();
    timer.set_freq(pwm_freq)?;

    let mut ch_u_pwm = timer.take_ch1()?.as_output(Ch1CompareMode::PwmMode1);
    ch_u_pwm.config_as_pwm();
    ch_u_pwm.set_duty_cycle(0f32)?;
    ch_u_pwm.complement_mut().set_deadtime(deadtime)?;
    ch_u_pwm.complement_mut().enable();

    let mut ch_v_pwm = timer.take_ch2()?.as_output(Ch2CompareMode::PwmMode1);
    ch_v_pwm.config_as_pwm();
    ch_v_pwm.set_duty_cycle(0f32)?;
    ch_v_pwm.complement_mut().set_deadtime(deadtime)?;
    ch_v_pwm.complement_mut().enable();

    let mut ch_w_pwm = timer.take_ch3()?.as_output(Ch3CompareMode::PwmMode1);
    ch_w_pwm.config_as_pwm();
    ch_w_pwm.set_duty_cycle(0f32)?;
    ch_w_pwm.complement_mut().set_deadtime(deadtime)?;
    ch_w_pwm.complement_mut().enable();

    Ok(Self {
      timer,
      ch_u_pwm,
      ch_v_pwm,
      ch_w_pwm,

      ch_u_pin: gpio_e.take_pe9()?.as_alt_func(
        PullDirection::Down,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      ch_v_pin: gpio_e.take_pe11()?.as_alt_func(
        PullDirection::Down,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      ch_w_pin: gpio_e.take_pe13()?.as_alt_func(
        PullDirection::Down,
        OutputType::PushPull,
        OutputSpeed::High,
      ),

      ch_un_pin: gpio_e.take_pe8()?.as_alt_func(
        PullDirection::Down,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      ch_vn_pin: gpio_e.take_pe10()?.as_alt_func(
        PullDirection::Down,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      ch_wn_pin: gpio_e.take_pe12()?.as_alt_func(
        PullDirection::Down,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      phase_angle: 0f32,
      power_scale: 0f32,
    })
  }

  pub fn get_phase_angle(&self) -> f32 {
    self.phase_angle
  }

  pub fn get_power_scale(&self) -> f32 {
    self.power_scale
  }

  pub fn set_power_scale(&mut self, power_scale: f32) -> Result<()> {
    self.set_phase_angle_and_power(self.phase_angle, power_scale)
  }

  pub fn set_phase_angle(&mut self, phase_angle: f32) -> Result<()> {
    self.set_phase_angle_and_power(norm_rads(phase_angle), self.power_scale)
  }

  pub fn set_phase_angle_and_power(&mut self, phase_angle: f32, power_scale: f32) -> Result<()> {
    let pa = norm_rads(phase_angle);

    let ps = match power_scale {
      s if s < 0f32 => 0f32,
      s if s > 1f32 => 1f32,
      _ => power_scale,
    };

    let u = Self::phase_angle_to_duty_cycle(pa);
    let v = Self::phase_angle_to_duty_cycle(norm_rads(pa - PI2_3));
    let w = Self::phase_angle_to_duty_cycle(norm_rads(pa - PI4_3));

    self.ch_u_pwm.set_duty_cycle(u * ps)?;
    self.ch_v_pwm.set_duty_cycle(v * ps)?;
    self.ch_w_pwm.set_duty_cycle(w * ps)?;

    self.phase_angle = pa;
    self.power_scale = ps;

    Ok(())
  }

  #[inline]
  pub fn phase_angle_to_duty_cycle(phase_angle: f32) -> f32 {
    libm::cosf(phase_angle) / 2f32 + 0.5
  }

  pub fn start(&mut self) {
    self.timer.start();
  }

  pub fn stop(&mut self) {
    self.timer.stop();
  }

  pub fn return_hardware(mut self, system: &mut System, gpio_e: &mut GpioE) -> Result<()> {
    self.stop();

    self.timer.return_ch1(self.ch_u_pwm.teardown()?)?;
    self.timer.return_ch2(self.ch_v_pwm.teardown()?)?;
    self.timer.return_ch3(self.ch_w_pwm.teardown()?)?;

    system.deactivate_tim1(self.timer)?;

    gpio_e.return_pe9(self.ch_u_pin.teardown())?;
    gpio_e.return_pe11(self.ch_v_pin.teardown())?;
    gpio_e.return_pe13(self.ch_w_pin.teardown())?;

    gpio_e.return_pe8(self.ch_un_pin.teardown())?;
    gpio_e.return_pe10(self.ch_vn_pin.teardown())?;
    gpio_e.return_pe12(self.ch_wn_pin.teardown())?;

    Ok(())
  }
}
