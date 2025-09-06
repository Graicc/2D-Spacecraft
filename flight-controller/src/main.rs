#![no_std]
#![no_main]

use core::panic::PanicInfo;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Speed},
    peripherals::{self, PA11, PA12, USB_OTG_FS},
    spi::Spi,
    time::Hertz,
    usb::{self, Driver},
};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Delay, Duration, Timer};
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, State},
};
use embedded_hal::{digital::OutputPin, spi::SpiBus};
use mpu6000::{MPU6000, accelerometer_sensitive, gyro_sensitive};

embassy_stm32::bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("{info}"); // TODO: this doesn't work
    cortex_m::asm::udf();
}

#[embassy_executor::task]
async fn logger_task(
    peri: Peri<'static, USB_OTG_FS>,
    dp: Peri<'static, PA12>,
    dm: Peri<'static, PA11>,
) {
    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb::Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    config.vbus_detection = false;

    let driver = Driver::new_fs(peri, Irqs, dp, dm, &mut ep_out_buffer, config);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // This has to be before usb fut
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Debug, class);

    // Run the USB device.
    let usb_fut = usb.run();

    join(usb_fut, log_fut).await;
}

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
        .spawn(logger_task(p.USB_OTG_FS, p.PA12, p.PA11))
        .unwrap();

    // Onboard led
    let mut led: Output<'_> = Output::new(p.PB5, Level::High, Speed::Low);

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(1_000_000);

    let spi1 = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_config);
    // let spi_bus = MPU6000::bus::SPIBus(spi1);
    let spi_bus =
        mpu6000::bus::SpiBus::new(spi1, Output::new(p.PA4, Level::Low, Speed::Low), Delay);
    let mut mpu6000 = MPU6000::new(spi_bus);
    let mut delay = Delay;
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

    loop {
        // led.set_high();
        // log::info!("Blink");
        let acc = mpu6000
            .read_acceleration()
            .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"));
        log::info!("{}, {}, {}", acc.0[0], acc.0[1], acc.0[2]);
        Timer::after(Duration::from_millis(0)).await;
        // led.set_low();
        // Timer::after(Duration::from_millis(100)).await;
    }
}
