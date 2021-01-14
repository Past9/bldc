use core::fmt::Write;
use stm32f303_api::{
  gpio::{
    gpio_a::{
      GpioA, Pa4Output, Pa5AltFunc, Pa5Spi1Sck, Pa6AltFunc, Pa6Spi1Miso, Pa7AltFunc, Pa7Spi1Mosi,
    },
    DigitalValue, OutputSpeed, OutputType, PullDirection,
  },
  spi::spi_i2s_1::{MasterRole, MotorolaFrameFormat, Spi, SpiProtocol},
  System,
};
use stm32f303_api::{
  spi::{BitOrder, ClockPhase, ClockPolarity},
  Result,
};

use crate::math::{norm_rads, PI1_2, PI2};

const POS_MAX_U16: u16 = 0b0011111111111111; // Max value of 14-bit position sensor
const POS_MAX_F32: f32 = POS_MAX_U16 as f32; // Max value of 14-bit position sensor

fn raw_to_rads(raw: u16) -> f32 {
  (raw as f32 / POS_MAX_F32) * PI2
}

pub struct PositionSensor {
  num_magnet_pairs: u32,
  offset: f32,
  rads_per_magnet_pair: f32,
  spi: Spi<SpiProtocol, MotorolaFrameFormat, MasterRole>,
  csn: Pa4Output,
  sck: Pa5AltFunc<Pa5Spi1Sck>,
  miso: Pa6AltFunc<Pa6Spi1Miso>,
  mosi: Pa7AltFunc<Pa7Spi1Mosi>,
  last_command: Command,
}
impl PositionSensor {
  pub fn new(num_magnet_pairs: u32, system: &mut System, gpio_a: &mut GpioA) -> Result<Self> {
    let mut spi: Spi<SpiProtocol, MotorolaFrameFormat, MasterRole> =
      system.activate_spi_i2s_1()?.as_spi();
    spi.enable_software_slave_management();
    spi.set_internal_slave_select();
    spi.set_bit_order(BitOrder::MsbFirst);
    spi.set_data_size(16)?;
    spi.set_clock_polarity(ClockPolarity::IdleLow);
    spi.set_clock_phase(ClockPhase::SecondTransition);

    Ok(Self {
      num_magnet_pairs,
      offset: 0f32,
      rads_per_magnet_pair: PI2 / num_magnet_pairs as f32,
      spi,
      csn: gpio_a
        .take_pa4()?
        .as_output(PullDirection::Up, OutputType::PushPull, OutputSpeed::High),
      sck: gpio_a.take_pa5()?.as_alt_func(
        PullDirection::Up,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      miso: gpio_a.take_pa6()?.as_alt_func(
        PullDirection::Down,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      mosi: gpio_a.take_pa7()?.as_alt_func(
        PullDirection::Up,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      last_command: Command::Nop,
    })
  }

  pub fn set_offset(&mut self, offset: f32) {
    println!("SET OFFSET").ok();
    self.offset = offset;
  }

  pub fn get_offset(&self) -> f32 {
    self.offset
  }

  pub fn start(&mut self) {
    self.csn.write(DigitalValue::High);
    self.spi.start();
  }

  pub fn stop(&mut self) -> Result<()> {
    self.spi.wait_for_not_busy()?;
    self.csn.write(DigitalValue::High);
    self.spi.stop();
    Ok(())
  }

  pub fn read_absolute_angle(&mut self) -> Result<f32> {
    let rads = raw_to_rads(self.read(ReadCommand::Angle)? & POS_MAX_U16);
    Ok(norm_rads(rads - self.offset))
  }

  pub fn read_phase_angle(&mut self) -> Result<f32> {
    Ok(
      (self.read_absolute_angle()? % (PI2 / self.num_magnet_pairs as f32))
        * self.num_magnet_pairs as f32,
    )
    //Ok(self.read_absolute_angle()? * self.num_magnet_pairs as f32)
  }

  pub fn read(&mut self, read_command: ReadCommand) -> Result<u16> {
    let command_previously_sent = match self.last_command {
      Command::Nop => false,
      Command::Read(rc) => rc == read_command,
      Command::Write(_) => false,
    };

    if !command_previously_sent {
      self.send(Command::Read(read_command))?;
    }

    self.send(Command::Read(read_command))
  }

  pub fn send(&mut self, command: Command) -> Result<u16> {
    self.csn.write(DigitalValue::Low);
    self.spi.write(match command {
      Command::Nop => 0,
      Command::Read(rc) => rc as u16,
      Command::Write(wc) => wc as u16,
    });
    self.spi.wait_for_not_busy()?;
    self.csn.write(DigitalValue::High);
    self.last_command = command;
    Ok(self.spi.read())
  }

  pub fn return_hardware(mut self, system: &mut System, gpio_a: &mut GpioA) -> Result<()> {
    self.stop()?;
    system.deactivate_spi_i2s_1(self.spi.teardown())?;
    gpio_a.return_pa4(self.csn.teardown())?;
    gpio_a.return_pa5(self.sck.teardown())?;
    gpio_a.return_pa6(self.miso.teardown())?;
    gpio_a.return_pa7(self.mosi.teardown())?;
    Ok(())
  }
}

#[derive(Copy, Clone, PartialEq)]
#[repr(u16)]
pub enum Command {
  Nop,
  Read(ReadCommand),
  Write(WriteCommand),
}

#[derive(Copy, Clone, PartialEq)]
pub enum ReadCommand {
  Errors = 0b0100000000000001,
  Diagnostics = 0b0111111111111101,
  Magnitude = 0b0111111111111110,
  Angle = 0b1111111111111111,
}

#[derive(Copy, Clone, PartialEq)]
pub enum WriteCommand {}
