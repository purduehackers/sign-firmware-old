use embassy_rp::spi::{Instance, Mode, Spi};

pub struct Eeprom<'a, T: Instance, M: Mode> {
    spi: Spi<'a, T, M>,
}

impl<T: Instance, M: Mode> Eeprom<'_, T, M> {
    pub fn new() -> Self {
        // Self {
        //     spi: Spi::new_blocking(inner, clk, mosi, miso, config)
        // }
        todo!()
    }
}
