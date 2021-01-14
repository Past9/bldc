pub const PI: f32 = 3.14159;
pub const PI2: f32 = PI * 2f32;
pub const PI1_2: f32 = PI / 2f32;
pub const PI1_4: f32 = PI / 4f32;

pub fn norm_rads(rads: f32) -> f32 {
  libm::fmodf(PI2 + libm::fmodf(rads, PI2), PI2)
  //rads
}
