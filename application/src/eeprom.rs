use core::mem::size_of;
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Blocking, Error, Spi};

const READ_INSTRUCTION: u32 = 0b0000_0011;
const WRITE_INSTRUCTION: u32 = 0b0000_0010;
const ERASE_INSTRUCTION: u32 = 0b1100_0111;
/// 0xdeadbeef 0x8acc8acc (deadbeef hacchacc)
const READ_WRITE_TEST: [u8; 8] = [0xde, 0xad, 0xbe, 0xef, 0x8a, 0xcc, 0x8a, 0xcc];
const METADATA_BLOCK_START: u32 = 0x10;
const DATA_BLOCK_START: u32 = 0x100;
const CONFIG_VERSION: u8 = 1;

pub struct Eeprom<'a> {
    spi: Spi<'a, SPI0, Blocking>,
    metadata: Metadata,
}

#[derive(Default, bincode::Decode, bincode::Encode)]
struct Metadata {
    config_version: u8,
    written_configs: u8,
}

const BINCODE_CONFIG: bincode::config::Configuration<bincode::config::BigEndian> =
    bincode::config::standard()
        .with_big_endian()
        .with_variable_int_encoding();

impl<'a> Eeprom<'a> {
    pub fn from_spi(spi: Spi<'a, SPI0, Blocking>) -> Result<Self, Error> {
        let mut eeprom = Self {
            spi,
            metadata: Default::default(),
        };

        assert!(eeprom.read_write_works()?, "Read-write check failed!");

        let mut metadata = [0x0; size_of::<Metadata>()];

        eeprom.read_bytes(METADATA_BLOCK_START, &mut metadata)?;

        let (metadata, _) = bincode::decode_from_slice::<Metadata, _>(&metadata, BINCODE_CONFIG)
            .expect("decode metadata");

        if metadata.config_version != CONFIG_VERSION {
            eeprom.erase()?;
            eeprom.write_config()?;
        }

        eeprom.metadata = metadata;

        Ok(eeprom)
    }

    fn write_config(&mut self) -> Result<(), Error> {
        let mut metadata = [0x0_u8, size_of::<Metadata>() as u8];

        bincode::encode_into_slice(&self.metadata, &mut metadata, BINCODE_CONFIG)
            .expect("encode metadata");

        self.write_bytes(METADATA_BLOCK_START, &metadata)
    }

    fn read_bytes(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), Error> {
        self.spi.blocking_write(&READ_INSTRUCTION.to_be_bytes())?;
        self.spi.blocking_write(&address.to_be_bytes()[1..])?;

        self.spi.blocking_read(buffer)?;

        Ok(())
    }

    fn write_bytes(&mut self, address: u32, buffer: &[u8]) -> Result<(), Error> {
        self.spi.blocking_write(&WRITE_INSTRUCTION.to_be_bytes())?;
        self.spi.blocking_write(&address.to_be_bytes()[1..])?;

        self.spi.blocking_write(buffer)?;

        Ok(())
    }

    fn erase(&mut self) -> Result<(), Error> {
        self.spi.blocking_write(&ERASE_INSTRUCTION.to_be_bytes())?;

        Ok(())
    }

    pub fn read_write_works(&mut self) -> Result<bool, Error> {
        self.write_bytes(0x0, &READ_WRITE_TEST)?;

        let mut buffer = [0x0; 8];

        self.read_bytes(0x0, &mut buffer)?;

        Ok(buffer == READ_WRITE_TEST)
    }
}
