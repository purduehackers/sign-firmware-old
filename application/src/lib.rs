#![no_std]

pub mod eeprom;
pub mod schema;

pub fn ceil(val: f32) -> f32 {
    (val as u32 + 1) as f32
}
