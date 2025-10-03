#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    spi::Spi,
    time::Hertz,
};
use embassy_time::{Delay, Duration, Ticker};
use mpu6000::MPU6000;

mod logger;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            // mode: HseMode::Bypass,
            mode: HseMode::Oscillator, // We want to use the internal oscillator! We don't have an external one
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
            divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);

    spawner
        .spawn(logger::logger_task(p.USB_OTG_FS, p.PA12, p.PA11))
        .unwrap();

    // Onboard led
    // let mut led: Output<'_> = Output::new(p.PB5, Level::High, Speed::Low);

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(1_000_000);

    let spi1 = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_config);
    // let spi_bus = MPU6000::bus::SPIBus(spi1);
    let spi_bus =
        mpu6000::bus::SpiBus::new(spi1, Output::new(p.PA4, Level::Low, Speed::Low), Delay);
    let mut mpu6000 = MPU6000::new(spi_bus);
    mpu6000
        .reset(&mut Delay)
        .unwrap_or_else(|_| panic!("Failed to reset MPU6000"));
    mpu6000
        .set_sleep(false)
        .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"));

    mpu6000
        .set_accelerometer_sensitive(mpu6000::registers::AccelerometerSensitive::Sensitive2048)
        .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"));
    mpu6000
        .set_gyro_sensitive(mpu6000::registers::GyroSensitive::Sensitive16_4)
        .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"));

    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        // led.set_high();
        // log::info!("Blink");
        let acc = mpu6000
            .read_acceleration()
            .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"))
            .0;
        let gyro = mpu6000
            .read_gyro()
            .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"))
            .0;
        log::info!(
            "{},{},{},{},{},{}",
            acc[0],
            acc[1],
            acc[2],
            gyro[0],
            gyro[1],
            gyro[2]
        );
        ticker.next().await;
        // led.set_low();
        // Timer::after(Duration::from_millis(100)).await;
    }
}
