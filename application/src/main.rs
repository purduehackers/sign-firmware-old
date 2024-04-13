#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Executor;
use embassy_rp::{
    gpio::{Level, Output, Pin},
    multicore::{spawn_core1, Stack},
    spi::{Spi, Config},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use application::eeprom;
use application::schema;

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     let _p = embassy_rp::init(Default::default());
//     loop {
//         defmt::info!("Blink");
//         Timer::after(Duration::from_millis(100)).await;
//     }
// }

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, LedState, 1> = Channel::new();

enum LedState {
    On,
    Off,
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let led = Output::new(p.PIN_25, Level::Low); 

    let mut config = Config::default();
    config.frequency = 20_000_000;
    let spi = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, config);
    let cs = Output::new(p.PIN_17, Level::High); 

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| unwrap!(spawner.spawn(core1_task(led, spi, cs))));
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| unwrap!(spawner.spawn(core0_task())));
}

enum AnimationSelection {
    LightningTime,
    Custom(u32),
}

#[embassy_executor::task]
async fn core0_task() {
    info!("CORE 0 START");
    loop {
        CHANNEL.send(LedState::On).await;
        Timer::after_millis(100).await;
        CHANNEL.send(LedState::Off).await;
        Timer::after_millis(400).await;
    }
}

#[embassy_executor::task]
async fn core1_task(mut led: Output<'static, impl Pin>, spi: Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Blocking>, cs: Output<'static, embassy_rp::peripherals::PIN_17>) {
    info!("CORE 1 START");
    let eeprom = eeprom::Eeprom::from_spi(spi, cs).await.expect("eeprom init");
    info!("EEPROM Initialized succesfully.");
    loop {
        match CHANNEL.receive().await {
            LedState::On => led.set_high(),
            LedState::Off => led.set_low(),
        }
    }
}
