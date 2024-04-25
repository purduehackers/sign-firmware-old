#![no_std]

use core::hint::unreachable_unchecked;

use embassy_rp::{
    peripherals::{
        PWM_SLICE0, PWM_SLICE1, PWM_SLICE2, PWM_SLICE3, PWM_SLICE4, PWM_SLICE5, PWM_SLICE6,
        PWM_SLICE7,
    },
    pwm::{Config, Pwm},
};

pub mod eeprom;
pub mod schema;

pub fn ceil(val: f32) -> f32 {
    (val as u32 + 1) as f32
}

pub struct Leds<'a> {
    pub s0: Pwm<'a, PWM_SLICE0>,
    pub s1: Pwm<'a, PWM_SLICE1>,
    pub s2: Pwm<'a, PWM_SLICE2>,
    pub s3: Pwm<'a, PWM_SLICE3>,
    pub s4: Pwm<'a, PWM_SLICE4>,
    pub s5: Pwm<'a, PWM_SLICE5>,
    pub s6: Pwm<'a, PWM_SLICE6>,
    pub s7: Pwm<'a, PWM_SLICE7>,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Block {
    Center = 1,
    BottomLeft,
    BottomRight,
    Right,
    Top,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum PwmSubchannel {
    A,
    B,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum Color {
    Red = 1,
    Green,
    Blue,
}

impl Block {
    fn channel_for_color(&self, color: Color) -> (u8, PwmSubchannel) {
        let sum = *self as u8 * color as u8 - 1;
        let channel = sum / 2;
        let subchannel = sum % 2;
        (
            channel,
            match subchannel {
                0 => PwmSubchannel::A,
                1 => PwmSubchannel::B,
                _ => unsafe { unreachable_unchecked() },
            },
        )
    }
}

impl Leds<'_> {
    pub fn set_color(&mut self, color: palette::Srgb<u8>, block: Block) {
        let (r_c, r_sc) = block.channel_for_color(Color::Red);
        self.set_color_channel(color.red, r_c, r_sc);
        let (g_c, g_sc) = block.channel_for_color(Color::Green);
        self.set_color_channel(color.green, g_c, g_sc);
        let (b_c, b_sc) = block.channel_for_color(Color::Blue);
        self.set_color_channel(color.blue, b_c, b_sc);
    }

    fn set_color_channel(&mut self, channel_val: u8, pwm_channel: u8, subchannel: PwmSubchannel) {
        let mut config = Config::default();
        let channel_val: u16 = channel_val as u16 * 257;
        match subchannel {
            PwmSubchannel::A => {
                config.compare_a = channel_val;
            }
            PwmSubchannel::B => {
                config.compare_b = channel_val;
            }
        }

        match pwm_channel {
            0 => self.s0.set_config(&config),
            1 => self.s1.set_config(&config),
            2 => self.s2.set_config(&config),
            3 => self.s3.set_config(&config),
            4 => self.s4.set_config(&config),
            5 => self.s5.set_config(&config),
            6 => self.s6.set_config(&config),
            7 => self.s7.set_config(&config),
            _ => unsafe { unreachable_unchecked() },
        }
    }
}
