use stm32f303_api::Result;

pub trait Program {
  fn step(&mut self) -> Result<()>;
  fn shutdown(self) -> Result<()>;
  fn should_continue(&mut self) -> Result<bool>;
  fn safemode(&mut self);
}

pub fn run<P: Program>(program: Result<P>) -> ! {
  match program {
    Err(e) => panic!(e.message),
    Ok(p) => {
      if let Err(e) = do_loop(p) {
        panic!(e.message);
      }
    }
  };

  loop {}
}

fn do_loop<P: Program>(mut program: P) -> Result<()> {
  while program.should_continue().map_err(|e| {
    program.safemode();
    e
  })? {
    program.step().map_err(|e| {
      program.safemode();
      e
    })?;
  }

  program.shutdown()?;

  Ok(())
}
