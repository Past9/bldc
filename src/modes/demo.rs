use stm32f303_api::Result;

use crate::{
  drv_8305::Drv8305, magnet_controller::MagnetController, math::PI, math::PI1_2, math::PI1_4,
  math::PI2, position_sensor::PositionSensor,
};

const MIN: f32 = 0f32;
const MAX: f32 = 0.2;

pub struct DemoMode {
  accel: f32,
  power: f32,
  angle: f32,
}
impl DemoMode {
  pub fn new(drv_8305: &mut Drv8305, magnet_controller: &mut MagnetController) -> Result<Self> {
    drv_8305.enable_gate();
    //magnet_controller.set_power_scale(0.2)?;
    magnet_controller.set_phase_angle_and_power(0f32, 0f32)?;
    Ok(Self {
      accel: 0.0001f32,
      power: MIN,
      angle: PI1_2,
    })
  }

  pub fn step(
    &mut self,
    drv_8305: &mut Drv8305,
    magnet_controller: &mut MagnetController,
    position_sensor: &mut PositionSensor,
  ) -> Result<()> {
    let phase_pos = position_sensor.read_phase_angle()?;

    if self.power > MAX || self.power < MIN {
      self.accel *= -1f32;
    }

    if self.power < MIN {
      self.angle *= -1f32;
    }

    self.power += self.accel;

    //current_controller.set_phase_angle(phase_pos + 0.5)?;
    magnet_controller.set_phase_angle_and_power(phase_pos + self.angle, self.power)?;
    Ok(())
  }
}
