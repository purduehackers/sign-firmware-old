#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{hint::unreachable_unchecked, ptr::addr_of_mut};

use chrono::{DateTime, Datelike, NaiveDateTime, NaiveTime, Timelike};
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Config as NetConfig, Stack as NetStack, StackResources,
};
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    multicore::{spawn_core1, Stack},
    peripherals::{DMA_CH0, PIO0, RTC},
    pio::{InterruptHandler, Pio},
    pwm::Pwm,
    rtc::{DateTime as RpDateTime, DayOfWeek, Rtc},
    spi::{Config, Spi},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use lightning_time::LightningTime;
use reqwless::client::HttpClient;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use application::{eeprom, Leds};
use application::{schema, Block};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const WIFI_SSID: &str = "PAL-Gadgets";

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static NET_STACK: StaticCell<NetStack<cyw43::NetDriver>> = StaticCell::new();
static mut NET_STACK_RESOURCES: StackResources<16> = StackResources::new();
static RTC_CHANNEL: Channel<CriticalSectionRawMutex, Rtc<'static, RTC>, 1> = Channel::new();

/*
PWM MAP:
Channel 0:
A: Center R
B: Center G
Channel 1:
A: Center B
B: BL R
Channel 2:
A: BL G
B: BL B
Channel 3:
A: BR R
B: BR G
Channel 4:
A: BR B
B: Right R
Channel 5:
A: Right G
B: Right B
Channel 6:
A: Top R
B: Top G
Channel 7:
A: Top B
B:
*/

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

    let rtc = Rtc::new(p.RTC);

    let config = embassy_rp::pwm::Config::default();

    let leds = Leds {
        s0: Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_0, p.PIN_1, config.clone()),
        s1: Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_2, p.PIN_3, config.clone()),
        s2: Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_4, p.PIN_5, config.clone()),
        s3: Pwm::new_output_ab(p.PWM_SLICE3, p.PIN_6, p.PIN_7, config.clone()),
        s4: Pwm::new_output_ab(p.PWM_SLICE4, p.PIN_8, p.PIN_9, config.clone()),
        s5: Pwm::new_output_ab(p.PWM_SLICE5, p.PIN_10, p.PIN_11, config.clone()),
        s6: Pwm::new_output_ab(p.PWM_SLICE6, p.PIN_12, p.PIN_13, config.clone()),
        s7: Pwm::new_output_a(p.PWM_SLICE7, p.PIN_14, config),
    };

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| unwrap!(spawner.spawn(core1_task(leds))));
        },
    );

    // This MUST be run on core 0 to avoid memory/stack-related freezes
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        unwrap!(spawner.spawn(core0_task(spi, cs, spawner, pwr, pio_spi, state, rtc)))
    });
}

enum AnimationSelection {
    LightningTime,
    Custom(u32),
}

#[embassy_executor::task]
async fn core1_task(mut leds: Leds<'static>) {
    info!("CORE 1 START");
    info!("Waiting for RTC...");
    let rtc = RTC_CHANNEL.receive().await;
    info!("RTC received!");
    loop {
        let now = rtc.now().expect("rtc now");
        let time = NaiveTime::from_hms_opt(now.hour as u32, now.minute as u32, now.second as u32)
            .expect("valid time");
        let time = LightningTime::from(time);
        let colors = time.colors();
        leds.set_color(colors.bolt, Block::BottomLeft);
        for block in [Block::Top, Block::Center] {
            leds.set_color(colors.zap, block);
        }
        for block in [Block::Right, Block::BottomRight] {
            leds.set_color(colors.spark, block);
        }
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[derive(Debug, serde::Deserialize)]
struct TimeResponse {
    unixtime: i64,
}

#[embassy_executor::task]
async fn core0_task(
    spi: Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Blocking>,
    cs: Output<'static>,
    spawner: Spawner,
    pwr: Output<'static>,
    pio_spi: PioSpi<'static, PIO0, 0, DMA_CH0>,
    state: &'static mut cyw43::State,
    mut rtc: Rtc<'static, embassy_rp::peripherals::RTC>,
) {
    info!("CORE 0 START");

    let fw = include_bytes!("../firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/43439A0_clm.bin");
    let (net_device, mut control, runner) = cyw43::new(state, pwr, pio_spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::Performance)
        .await;

    info!(
        "Wi-Fi Initialized with MAC address {:02x}",
        control.address().await
    );

    let connected = match control.join_open(WIFI_SSID).await {
        Ok(()) => {
            info!("Wi-Fi connected!");
            true
        }
        Err(_e) => {
            warn!("Wi-Fi connection error!");
            false
        }
    };

    let stack = NetStack::new(
        net_device,
        NetConfig::default(),
        unsafe { &mut *addr_of_mut!(NET_STACK_RESOURCES) },
        0xdeadbeef,
    );
    let stack = NET_STACK.init(stack);
    let state: TcpClientState<16, 16, 16> = TcpClientState::new();
    let tcp = TcpClient::new(stack, &state);
    let dns = DnsSocket::new(stack);
    let mut client = HttpClient::new(&tcp, &dns);

    // Grab necessary time and update data
    let current_time = if connected {
        let mut headers = [0_u8; 1024];
        let mut time = client
            .request(
                reqwless::request::Method::GET,
                "http://worldtimeapi.org/api/timezone/America/New_York",
            )
            .await
            .expect("get time");
        let time = time.send(&mut headers).await.expect("time response");

        let body = time.body().read_to_end().await.expect("read body");

        let time: TimeResponse = serde_json_core::from_slice(body).expect("valid response").0;
        let time = DateTime::from_timestamp(time.unixtime, 0).expect("valid unix time");

        RpDateTime {
            year: time.year() as u16,
            day: time.day() as u8,
            month: time.month() as u8,
            day_of_week: match time.weekday().num_days_from_sunday() {
                0 => DayOfWeek::Sunday,
                1 => DayOfWeek::Monday,
                2 => DayOfWeek::Tuesday,
                3 => DayOfWeek::Wednesday,
                4 => DayOfWeek::Thursday,
                5 => DayOfWeek::Friday,
                6 => DayOfWeek::Saturday,
                _ => unsafe { unreachable_unchecked() },
            },
            hour: time.hour() as u8,
            minute: time.minute() as u8,
            second: time.second() as u8,
        }
    } else {
        RpDateTime {
            year: 2003,
            day: 29,
            month: 12,
            day_of_week: embassy_rp::rtc::DayOfWeek::Monday,
            hour: 0,
            minute: 0,
            second: 0,
        }
    };

    rtc.set_datetime(current_time).expect("set datetime");

    // This is so janky but I think this is the safest way to send stuff across threads like this
    RTC_CHANNEL.send(rtc).await;

    let eeprom = eeprom::Eeprom::from_spi(spi, cs)
        .await
        .expect("eeprom init");
    info!("EEPROM Initialized succesfully.");
    loop {
        // match CHANNEL.receive().await {
        //     LedState::On => info!("LED High"),
        //     LedState::Off => info!("LED low"),
        // }
        Timer::after_secs(1).await;
    }
}
