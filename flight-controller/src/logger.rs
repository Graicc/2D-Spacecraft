use core::panic::PanicInfo;
use embassy_stm32::peripherals::USB_OTG_FS;


#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("{info}"); // TODO: this doesn't work
    // cortex_m::asm::udf();
    cortex_m::asm::delay(1_000_000);
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::task]
pub async fn logger_task(driver: embassy_stm32::usb::Driver<'static, USB_OTG_FS>) -> ! {
    embassy_usb_logger::run!(1024, log::LevelFilter::Trace, driver);
}
