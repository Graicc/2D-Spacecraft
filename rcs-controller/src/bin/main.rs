#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, gpio::Output};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_println::println;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    let mut fan1 = Output::new(
        peripherals.GPIO14,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut fan2 = Output::new(
        peripherals.GPIO27,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut fan3 = Output::new(
        peripherals.GPIO26,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut fan4 = Output::new(
        peripherals.GPIO25,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut fan5 = Output::new(
        peripherals.GPIO33,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut fan6 = Output::new(
        peripherals.GPIO32,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );

    let mut fans = [fan1, fan2, fan3, fan4, fan5, fan6];

    // let mut fan = fan1;
    loop {
        for fan in &mut fans {
            fan.set_low();
        }
        println!("low");
        Timer::after(Duration::from_secs(3)).await;
        println!("high");
        for fan in &mut fans {
            // fan.set_high();
            fan.set_low();
        }
        Timer::after(Duration::from_secs(3)).await;
        // fan.set_low();
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}
