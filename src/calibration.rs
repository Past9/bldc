use crate::{
  current_controller::CurrentController, drv_8305::Drv8305, position_sensor::PositionSensor, PI2,
};
use core::fmt::Write;
use stm32f303_api::Result;

const MAX_DEVIATION: f32 = PI2 / 10000f32;
const NUM_SAMPLES: usize = 500;
const SPEED: f32 = 0.002;
const MAX_TURN: f32 = PI2 * 5f32;

enum Phase {
  Start,
  Settle,
  ForwardTurn,
  ForwardSettle,
  BackwardTurn,
  BackwardSettle,
  Done,
}

pub struct CalibrationMode {
  phase: Phase,
  zero: f32,
  forward_extent: f32,
  backward_extent: f32,
  settler: Settler,
  cumulative_phase_angle: f32,
}
impl CalibrationMode {
  pub fn new() -> Result<Self> {
    Ok(Self {
      phase: Phase::Start,
      zero: 0f32,
      forward_extent: 0f32,
      backward_extent: 0f32,
      settler: Settler::new(),
      cumulative_phase_angle: 0f32,
    })
  }

  pub fn step(
    &mut self,
    drv_8305: &mut Drv8305,
    current_controller: &mut CurrentController,
    position_sensor: &mut PositionSensor,
  ) -> Result<()> {
    match self.phase {
      Phase::Start => {
        drv_8305.start();
        drv_8305.enable_gate();
        current_controller.set_angle_and_power(0f32, 0.1f32)?;
        self.phase = Phase::Settle;
      }
      Phase::Settle => {
        if let SettleState::Settled(zero) =
          self.settler.add_sample(position_sensor.read_position()?)
        {
          self.zero = zero;
          println!("Found zero at {} radians", self.zero).ok();
          self.phase = Phase::ForwardTurn;
        }
      }
      Phase::ForwardTurn => {
        self.cumulative_phase_angle += SPEED;
        current_controller.set_angle(self.cumulative_phase_angle)?;
        if self.cumulative_phase_angle >= MAX_TURN {
          current_controller.set_angle(MAX_TURN)?;
          self.settler = Settler::new();
          self.phase = Phase::ForwardSettle;
        }
      }
      Phase::ForwardSettle => {
        if let SettleState::Settled(forward_extent) =
          self.settler.add_sample(position_sensor.read_position()?)
        {
          self.forward_extent = forward_extent;
          //println!("Found forward extent at {} radians", self.forward_extent).ok();
          self.phase = Phase::BackwardTurn;
        }
      }
      Phase::BackwardTurn => {
        self.cumulative_phase_angle -= SPEED;
        current_controller.set_angle(self.cumulative_phase_angle)?;
        if self.cumulative_phase_angle <= 0f32 {
          self.settler = Settler::new();
          self.phase = Phase::BackwardSettle;
        }
      }
      Phase::BackwardSettle => {
        if let SettleState::Settled(backward_extent) =
          self.settler.add_sample(position_sensor.read_position()?)
        {
          self.backward_extent = backward_extent;
          //println!("Found backward extent at {} radians", self.backward_extent).ok();
          self.phase = Phase::Done;
        }
      }
      Phase::Done => {
        current_controller.set_angle_and_power(0f32, 0f32)?;
        drv_8305.disable_gate();
      }
    };

    Ok(())
  }
}

enum SettleState {
  Settled(f32),
  NotSettled,
}

struct Settler {
  samples_collected: usize,
  samples: [f32; NUM_SAMPLES],
}
impl Settler {
  pub fn new() -> Self {
    Self {
      samples_collected: 0,
      samples: [0f32; NUM_SAMPLES],
    }
  }

  pub fn last_sample(&self) -> f32 {
    self.samples[0]
  }

  pub fn add_sample(&mut self, sample: f32) -> SettleState {
    for i in 0..NUM_SAMPLES - 1 {
      self.samples[i + 1] = self.samples[i];
    }

    self.samples[0] = sample;
    self.samples_collected += 1;

    if self.samples_collected >= NUM_SAMPLES {
      let mut mean = 0f32;
      for i in 0..NUM_SAMPLES {
        mean += self.samples[i];
      }
      mean = mean / NUM_SAMPLES as f32;

      for i in 0..NUM_SAMPLES {
        if libm::fabsf(mean - self.samples[i]) > MAX_DEVIATION {
          return SettleState::NotSettled;
        }
      }

      return SettleState::Settled(mean);
    }

    SettleState::NotSettled
  }
}
