use core::panic::PanicInfo;
use embassy_futures::join::join;
use embassy_stm32::{
    peripherals::{self},
    usb::{self, Driver},
};
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, State},
};

use crate::LoggerResource;

embassy_stm32::bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("{info}"); // TODO: this doesn't work
    cortex_m::asm::udf();
}

#[embassy_executor::task]
pub async fn logger_task(res: LoggerResource) {
    let LoggerResource { peri, dp, dm } = res;

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
    let class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // This has to be before usb fut
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Debug, class);

    // Run the USB device.
    let usb_fut = usb.run();

    join(usb_fut, log_fut).await;
}
