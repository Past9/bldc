use stm32f303_api::Result;

use crate::{
  drv_8305::Drv8305, magnet_controller::MagnetController, position_sensor::PositionSensor,
};

pub struct RecoveryMode {}
impl RecoveryMode {
  pub fn new() -> Self {
    Self {}
  }

  pub fn step(
    &mut self,
    _drv_8305: &mut Drv8305,
    _current_controller: &mut MagnetController,
    _position_sensor: &mut PositionSensor,
  ) -> Result<()> {
    Ok(())
  }
}
