use stm32f303_api::{
  gpio::gpio_b::Pb11Output,
  gpio::gpio_b::{
    GpioB, Pb12Output, Pb13AltFunc, Pb13Spi2SckI2s2Ck, Pb14AltFunc, Pb14Spi2MisoI2s2extSd,
    Pb15AltFunc, Pb15Spi2MosiI2s2Sd,
  },
  gpio::DigitalValue,
  gpio::OutputSpeed,
  gpio::OutputType,
  gpio::PullDirection,
  spi::BitOrder,
  spi::ClockPolarity,
  spi::{spi_i2s_2::*, ClockPhase},
  Error, Result, System,
};

pub struct Drv8305 {
  en_gate: Pb11Output,
  spi: Spi<SpiProtocol, MotorolaFrameFormat, MasterRole>,
  csn: Pb12Output,
  sck: Pb13AltFunc<Pb13Spi2SckI2s2Ck>,
  miso: Pb14AltFunc<Pb14Spi2MisoI2s2extSd>,
  mosi: Pb15AltFunc<Pb15Spi2MosiI2s2Sd>,
  last_command: Command,
}
impl Drv8305 {
  pub fn new(system: &mut System, gpio_b: &mut GpioB) -> Result<Self> {
    let mut spi = system.activate_spi_i2s_2()?.as_spi();

    spi.enable_software_slave_management();
    spi.set_internal_slave_select();
    spi.set_bit_order(BitOrder::MsbFirst);
    spi.set_data_size(16)?;
    spi.set_clock_polarity(ClockPolarity::IdleLow);
    spi.set_clock_phase(ClockPhase::SecondTransition);

    Ok(Self {
      en_gate: gpio_b.take_pb11()?.as_output(
        PullDirection::Up,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      spi,
      csn: gpio_b.take_pb12()?.as_output(
        PullDirection::Up,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      sck: gpio_b.take_pb13()?.as_alt_func(
        PullDirection::Up,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      miso: gpio_b.take_pb14()?.as_alt_func(
        PullDirection::Up,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      mosi: gpio_b.take_pb15()?.as_alt_func(
        PullDirection::Up,
        OutputType::PushPull,
        OutputSpeed::High,
      ),
      last_command: Command::Nop,
    })
  }

  pub fn start(&mut self) {
    self.csn.write(DigitalValue::High);
    self.spi.start();
  }

  pub fn enable_gate(&mut self) {
    self.en_gate.write(DigitalValue::High);
  }

  pub fn disable_gate(&mut self) {
    self.en_gate.write(DigitalValue::Low);
  }

  pub fn stop(&mut self) -> Result<()> {
    self.disable_gate();
    self.spi.wait_for_not_busy()?;
    self.csn.write(DigitalValue::High);
    self.spi.stop();
    Ok(())
  }

  pub fn read_warnings(&mut self) -> Result<Warnings> {
    Warnings::decode(self.read(ReadCommand::Warnings)?)
  }

  pub fn read_overcurrent_faults(&mut self) -> Result<OvercurrentFaults> {
    OvercurrentFaults::decode(self.read(ReadCommand::OvercurrentFaults)?)
  }

  pub fn read_ic_faults(&mut self) -> Result<IcFaults> {
    IcFaults::decode(self.read(ReadCommand::IcFaults)?)
  }

  pub fn read_gate_driver_faults(&mut self) -> Result<GateDriverFaults> {
    GateDriverFaults::decode(self.read(ReadCommand::GateDriverFaults)?)
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

  pub fn return_hardware(mut self, system: &mut System, gpio_b: &mut GpioB) -> Result<()> {
    self.stop()?;
    self.spi.stop();

    system.deactivate_spi_i2s_2(self.spi.teardown())?;

    gpio_b.return_pb11(self.en_gate.teardown())?;
    gpio_b.return_pb12(self.csn.teardown())?;
    gpio_b.return_pb13(self.sck.teardown())?;
    gpio_b.return_pb14(self.miso.teardown())?;
    gpio_b.return_pb15(self.mosi.teardown())?;

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
#[repr(u16)]
pub enum ReadCommand {
  Warnings = 0b10001 << 11,
  OvercurrentFaults = 0b10010 << 11,
  IcFaults = 0b10011 << 11,
  GateDriverFaults = 0b0100 << 11,
}

#[derive(Copy, Clone, PartialEq)]
pub enum WriteCommand {}

#[repr(u16)]
pub enum WarningFlag {
  Overtemp = 1 << 0,
  TempOver135C = 1 << 1,
  TempOver125C = 1 << 2,
  TempOver105C = 1 << 3,
  ChargePumpUndervolt = 1 << 4,
  VdsOvercurrent = 1 << 5,
  PvddOvervolt = 1 << 6,
  PvddUndervolt = 1 << 7,
  TempOver175C = 1 << 8,
  Fault = 1 << 10,
}

pub struct Warnings {
  pub data: u16,
}
impl Warnings {
  pub fn decode(data: u16) -> Result<Self> {
    match data == core::u16::MAX {
      true => Err(Error::new("Drv8305 offline")),
      false => Ok(Self { data }),
    }
  }

  pub fn ok(&self) -> bool {
    self.data == 0
  }

  pub fn has(&self, flag: WarningFlag) -> bool {
    (self.data & (flag as u16)) > 0
  }
}

#[repr(u16)]
pub enum OvercurrentFaultFlag {
  SenseA = 1 << 0,
  SenseB = 1 << 1,
  SenseC = 1 << 2,
  MosfetLowC = 1 << 5,
  MosfetHighC = 1 << 6,
  MosfetLowB = 1 << 7,
  MosfetHighB = 1 << 8,
  MosfetLowA = 1 << 9,
  MosfetHighA = 1 << 10,
}

pub struct OvercurrentFaults {
  data: u16,
}
impl OvercurrentFaults {
  pub fn decode(data: u16) -> Result<Self> {
    match data == core::u16::MAX {
      true => Err(Error::new("Drv8305 offline")),
      false => Ok(Self { data }),
    }
  }

  pub fn ok(&self) -> bool {
    self.data == 0
  }

  pub fn has(&self, flag: OvercurrentFaultFlag) -> bool {
    self.data & flag as u16 > 0
  }
}

#[repr(u16)]
pub enum IcFaultFlag {
  HighSideChargePumpOvervoltAbs = 1 << 0,
  HighSideChargePumpOvervolt = 1 << 1,
  HighSideChargePumpUndervolt2 = 1 << 2,
  LowSideGateSupply = 1 << 4,
  AvddUndervolt = 1 << 5,
  VregUndervolt = 1 << 6,
  Overtemp = 1 << 8,
  Watchdog = 1 << 9,
  PvddUndervolt2 = 1 << 10,
}

pub struct IcFaults {
  data: u16,
}
impl IcFaults {
  pub fn decode(data: u16) -> Result<Self> {
    match data == core::u16::MAX {
      true => Err(Error::new("Drv8305 offline")),
      false => Ok(Self { data }),
    }
  }

  pub fn ok(&self) -> bool {
    self.data == 0
  }

  pub fn has(&self, flag: IcFaultFlag) -> bool {
    self.data & flag as u16 > 0
  }
}

#[repr(u16)]
pub enum GateDriverFaultFlag {
  LowMosfetC = 1 << 5,
  HighMosfetC = 1 << 6,
  LowMosfetB = 1 << 7,
  HighMosfetB = 1 << 8,
  LowMosfetA = 1 << 9,
  HighMosfetA = 1 << 10,
}

pub struct GateDriverFaults {
  data: u16,
}
impl GateDriverFaults {
  pub fn decode(data: u16) -> Result<Self> {
    match data == core::u16::MAX {
      true => Err(Error::new("Drv8305 offline")),
      false => Ok(Self { data }),
    }
  }

  pub fn ok(&self) -> bool {
    self.data == 0
  }

  pub fn has(&self, flag: GateDriverFaultFlag) -> bool {
    self.data & flag as u16 > 0
  }

  pub fn read_command() -> u16 {
    0b10100 << 15
  }
}
