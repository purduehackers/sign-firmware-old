#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output, Pin},
    multicore::{spawn_core1, Stack},
    peripherals::{DMA_CH0, PIO0},
    pio::{InterruptHandler, Pio},
    spi::{Config, Spi},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use application::eeprom;
use application::schema;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const WIFI_SSID: &str = env!("WIFI_SSID");

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

    let mut config = Config::default();
    config.frequency = 20_000_000;
    let spi = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, config);
    let cs = Output::new(p.PIN_17, Level::High);

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs_pio = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let pio_spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs_pio,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1
                .run(|spawner| unwrap!(spawner.spawn(core1_task())));
        },
    );

    // This MUST be run on core 0 to avoid memory/stack-related freezes
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| unwrap!(spawner.spawn(core0_task(spi, cs, spawner, pwr, pio_spi, state))));
}

enum AnimationSelection {
    LightningTime,
    Custom(u32),
}

#[embassy_executor::task]
async fn core1_task() {
    info!("CORE 1 START");
    loop {
        CHANNEL.send(LedState::On).await;
        Timer::after_millis(100).await;
        CHANNEL.send(LedState::Off).await;
        Timer::after_millis(400).await;
    }
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn core0_task(
    spi: Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Blocking>,
    cs: Output<'static>,
    spawner: Spawner,
    pwr: Output<'static>,
    pio_spi: PioSpi<'static, PIO0, 0, DMA_CH0>,
    state: &'static mut cyw43::State,
) {
    info!("CORE 0 START");

    let fw = include_bytes!("../firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/43439A0_clm.bin");
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, pio_spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::Performance)
        .await;

    info!(
        "WiFi Initialized with MAC address {:02x}",
        control.address().await
    );

    let eeprom = eeprom::Eeprom::from_spi(spi, cs)
        .await
        .expect("eeprom init");
    info!("EEPROM Initialized succesfully.");
    loop {
        match CHANNEL.receive().await {
            LedState::On => info!("LED High"),
            LedState::Off => info!("LED low"),
        }
    }
}
