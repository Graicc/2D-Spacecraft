#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use crate::{control::control_task, messaging::messaging_task};
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use {esp_backtrace as _, esp_println as _};

mod control;
mod messaging;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    let timer0 = TimerGroup::new(p.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    spawner
        .spawn(messaging_task(p.UART0, p.GPIO21, p.GPIO22))
        .unwrap();

    spawner
        .spawn(control_task(
            p.LEDC, p.GPIO14, p.GPIO27, p.GPIO26, p.GPIO25, p.GPIO33, p.GPIO32,
        ))
        .unwrap();
}
