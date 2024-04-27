#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{cell::RefCell, cmp::Ordering, hint::unreachable_unchecked, ptr::addr_of_mut};

use chrono::{DateTime, Datelike, NaiveDateTime, NaiveTime, Timelike};
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_boot_rp::*;
use embassy_executor::{Executor, Spawner};
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Config as NetConfig, ConfigV4, DhcpConfig, Stack as NetStack, StackResources,
};
use embassy_rp::{
    bind_interrupts,
    flash::{Blocking, Flash},
    gpio::{Level, Output},
    multicore::{spawn_core1, Stack},
    peripherals::{DMA_CH0, PIO0, RTC},
    pio::{InterruptHandler, Pio},
    pwm::Pwm,
    rtc::{DateTime as RpDateTime, DayOfWeek, Rtc},
    spi::{Config, Spi},
    watchdog::Watchdog,
};
use embassy_sync::{
    blocking_mutex::{
        raw::{CriticalSectionRawMutex, NoopRawMutex},
        Mutex,
    },
    channel::Channel,
};
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use lightning_time::LightningTime;
use reqwless::{
    client::{HttpClient, TlsConfig, TlsVerify},
    request::RequestBuilder,
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use application::{eeprom, Leds};
use application::{schema, Block};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const WIFI_SSID: &str = "PAL-Gadgets";
const CURRENT_VERSION_MAJOR: &str = env!("CARGO_PKG_VERSION_MAJOR");
const CURRENT_VERSION_MINOR: &str = env!("CARGO_PKG_VERSION_MINOR");
const CURRENT_VERSION_PATCH: &str = env!("CARGO_PKG_VERSION_PATCH");
const FLASH_SIZE: usize = 2 * 1024 * 1024;

static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static NET_STACK: StaticCell<NetStack<cyw43::NetDriver>> = StaticCell::new();
static mut NET_STACK_RESOURCES: StackResources<16> = StackResources::new();
static RTC_CHANNEL: Channel<CriticalSectionRawMutex, Rtc<'static, RTC>, 1> = Channel::new();
static CHANNEL: Channel<CriticalSectionRawMutex, LedState, 1> = Channel::new();

enum LedState {
    On,
    Off,
}

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

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(8));

    let mut config = Config::default();
    config.frequency = 20_000_000;
    let spi = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, config);
    let cs = Output::new(p.PIN_17, Level::High);

    let button_led = Output::new(p.PIN_20, Level::Low);

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

    let flash = Flash::<_, _, FLASH_SIZE>::new_blocking(p.FLASH);
    let flash: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(flash));

    let config = embassy_rp::pwm::Config::default();

    let leds = Leds {
        s0: Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_0, p.PIN_1, config.clone()).into(),
        s1: Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_2, p.PIN_3, config.clone()).into(),
        s2: Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_4, p.PIN_5, config.clone()).into(),
        s3: Pwm::new_output_ab(p.PWM_SLICE3, p.PIN_6, p.PIN_7, config.clone()).into(),
        s4: Pwm::new_output_ab(p.PWM_SLICE4, p.PIN_8, p.PIN_9, config.clone()).into(),
        s5: Pwm::new_output_ab(p.PWM_SLICE5, p.PIN_10, p.PIN_11, config.clone()).into(),
        s6: Pwm::new_output_ab(p.PWM_SLICE6, p.PIN_12, p.PIN_13, config.clone()).into(),
        s7: Pwm::new_output_a(p.PWM_SLICE7, p.PIN_14, config).into(),
    };

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| unwrap!(spawner.spawn(core1_task(leds, watchdog))));
        },
    );

    // This MUST be run on core 0 to avoid memory/stack-related freezes
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        unwrap!(spawner.spawn(core0_task(
            spi, cs, spawner, pwr, pio_spi, state, rtc, button_led, flash
        )))
    });
}

enum AnimationSelection {
    LightningTime,
    Custom(u32),
}

#[embassy_executor::task]
async fn core1_task(mut leds: Leds<'static>, mut watchdog: Watchdog) {
    info!("CORE 1 START");
    info!("Waiting for RTC...");
    let mut rtc;
    loop {
        rtc = RTC_CHANNEL.try_receive().ok();
        if rtc.is_some() {
            break;
        }
        watchdog.feed();
        Timer::after_millis(100).await
    }
    let rtc = rtc.expect("must have rtc");
    info!("RTC received!");
    CHANNEL.send(LedState::On).await;
    let mut log_counter = 0;
    loop {
        let now = rtc.now().expect("rtc now");
        let mut time =
            NaiveTime::from_hms_opt(now.hour as u32, now.minute as u32, now.second as u32)
                .expect("valid time");
        // Eastern time
        time -= chrono::Duration::hours(4);
        let time = LightningTime::from(time);

        log_counter += 1;
        if log_counter == 10 {
            trace!(
                "TIME: {:x}~{:x}~{:x}|{:x}",
                time.bolts,
                time.zaps,
                time.sparks,
                time.charges
            );
            log_counter = 0;
        }

        let colors = time.colors();

        leds.set_color(colors.bolt, Block::BottomLeft);

        for block in [Block::Top, Block::Center] {
            leds.set_color(colors.zap, block);
        }

        for block in [Block::Right, Block::BottomRight] {
            leds.set_color(colors.spark, block);
        }

        watchdog.feed();

        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static NetStack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[derive(Debug, serde::Deserialize)]
struct TimeResponse {
    unixtime: i64,
}

#[derive(Debug, serde::Deserialize)]
struct GithubAsset {
    browser_download_url: heapless::String<256>,
}

#[derive(Debug, serde::Deserialize)]
struct GithubResponse {
    tag_name: heapless::String<64>,
    assets: [GithubAsset; 1],
}

#[allow(clippy::too_many_arguments)]
#[embassy_executor::task(pool_size = 3)]
async fn core0_task(
    spi: Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Blocking>,
    cs: Output<'static>,
    spawner: Spawner,
    pwr: Output<'static>,
    pio_spi: PioSpi<'static, PIO0, 0, DMA_CH0>,
    state: &'static mut cyw43::State,
    mut rtc: Rtc<'static, embassy_rp::peripherals::RTC>,
    mut button_led: Output<'static>,
    flash: Mutex<
        NoopRawMutex,
        RefCell<Flash<'static, embassy_rp::peripherals::FLASH, Blocking, FLASH_SIZE>>,
    >,
) {
    info!("CORE 0 START");

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
    let mut aligned = AlignedBuffer([0; 1]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);

    updater.mark_booted().expect("mark booted");

    let fw = include_bytes!("../firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/43439A0_clm.bin");
    let (net_device, mut control, runner) = cyw43::new(state, pwr, pio_spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    info!(
        "Wi-Fi Initialized with MAC address {:02x}",
        control.address().await
    );

    let mut net_config = NetConfig::default();
    net_config.ipv4 = ConfigV4::Dhcp(DhcpConfig::default());

    let seed: u64 = 0x0123456789abcdef;

    let stack = NetStack::new(
        net_device,
        net_config,
        unsafe { &mut *addr_of_mut!(NET_STACK_RESOURCES) },
        seed,
    );
    let stack = &*NET_STACK.init(stack);

    unwrap!(spawner.spawn(net_task(stack)));

    info!("Attempting Wi-Fi connection...");
    let connected = match control.join_wpa2("Jah", "12345678").await {
        Ok(()) => {
            info!("Wi-Fi connected!");
            button_led.set_high();

            info!("Waiting for DHCP...");
            while !stack.is_config_up() {
                Timer::after_millis(100).await;
            }
            info!(
                "DHCP is now up! IP is {}",
                stack.config_v4().expect("v4 config").address.address()
            );
            true
        }
        Err(e) => {
            warn!("Wi-Fi connection error! Error: {}", e.status);
            false
        }
    };

    let state: TcpClientState<1, 2048, 2048> = TcpClientState::new();
    let tcp = TcpClient::new(stack, &state);
    let dns = DnsSocket::new(stack);
    let mut read_buffer = [0; 20_000];
    let mut write_buffer = [0; 20_000];
    let tls = TlsConfig::new(seed, &mut read_buffer, &mut write_buffer, TlsVerify::None);
    let mut client = HttpClient::new_with_tls(&tcp, &dns, tls);

    info!("HTTP Client initialized");

    // Grab necessary time and update data
    let current_time = if connected {
        let mut headers = [0_u8; 2048];
        let mut time = client
            .request(
                reqwless::request::Method::GET,
                "https://worldtimeapi.org/api/timezone/America/New_York",
            )
            .await
            .expect("get time");
        let time = time.send(&mut headers).await.expect("time response");
        let body = time.body().read_to_end().await.expect("read time body");
        let time: TimeResponse = serde_json_core::from_slice(body).expect("valid response").0;

        info!("Got time: {}", time.unixtime);

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

    // Updates
    if connected {
        let gh_response = {
            let mut headers = [0_u8; 0x2000];
            let mut release = client
                .request(
                    reqwless::request::Method::GET,
                    "https://api.github.com/repos/purduehackers/sign-firmware/releases/latest",
                )
                .await
                .expect("get release")
                .headers(&[("User-Agent", "PHSign/1.0.0"), ("Host", "api.github.com")]);

            let release = release.send(&mut headers).await.expect("release response");

            let body = release
                .body()
                .read_to_end()
                .await
                .expect("read release body");

            let gh_response: GithubResponse = serde_json_core::from_slice(body)
                .expect("valid gh response")
                .0;

            gh_response
        };

        let mut period_split = gh_response.tag_name.split('.');
        let major_remote: usize = period_split
            .next()
            .expect("major")
            .parse()
            .expect("major number");
        let minor_remote: usize = period_split
            .next()
            .expect("minor")
            .parse()
            .expect("minor number");
        let patch_remote: usize = period_split
            .next()
            .expect("patch")
            .parse()
            .expect("patch number");
        let major_local: usize = CURRENT_VERSION_MAJOR.parse().expect("major number");
        let minor_local: usize = CURRENT_VERSION_MINOR.parse().expect("minor number");
        let patch_local: usize = CURRENT_VERSION_PATCH.parse().expect("patch number");

        let needs_update = match major_local.cmp(&major_remote) {
            Ordering::Less => true,
            Ordering::Greater => false,
            Ordering::Equal => match minor_local.cmp(&minor_remote) {
                Ordering::Less => true,
                Ordering::Greater => false,
                Ordering::Equal => patch_local < patch_remote,
            },
        };

        if needs_update {
            let mut buffer = [0_u8; 4096];
            let mut artifact = client
                .request(
                    reqwless::request::Method::GET,
                    &gh_response.assets[0].browser_download_url,
                )
                .await
                .expect("get release");
            let artifact = artifact.send(&mut buffer).await.expect("release response");
            let mut artifact_reader = artifact.body().reader();
            let mut buffer = [0_u8; 4096];
            let mut read = 0;

            // Firmware updating stuffs
            let mut buf: AlignedBuffer<4096> = AlignedBuffer([0; 4096]);

            loop {
                read = artifact_reader
                    .read(&mut buffer)
                    .await
                    .expect("read artifact");

                if read == 0 {
                    break;
                }
            }
        }
    }

    let eeprom = eeprom::Eeprom::from_spi(spi, cs)
        .await
        .expect("eeprom init");
    info!("EEPROM Initialized succesfully.");
    loop {
        match CHANNEL.receive().await {
            LedState::On => control.gpio_set(0, true).await,
            LedState::Off => control.gpio_set(0, false).await,
        }
        Timer::after_millis(10).await;
    }
}
