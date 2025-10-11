use defmt::println;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::peripherals::{GPIO21, GPIO22, UART0};
use messages::{Message, Receiver};

pub static CURRENT_CMD_VEL: Signal<CriticalSectionRawMutex, messages::CMDVelValues> = Signal::new();

#[embassy_executor::task]
pub async fn messaging_task(uart: UART0<'static>, rx: GPIO21<'static>, tx: GPIO22<'static>) {
    let mut uart = esp_hal::uart::Uart::new(uart, Default::default())
        .unwrap()
        .with_rx(rx)
        .with_tx(tx)
        .into_async();

    let mut receiver: Receiver<Message, 256, 32> = messages::Receiver::new();

    loop {
        if let Ok(data) = receiver.receive(&mut uart).await {
            match data {
                Message::CMDVel(cmd_vel) => {
                    println!(
                        "Got current vel command: vx={} vy={} vt={}",
                        cmd_vel.vx, cmd_vel.vy, cmd_vel.vt
                    );
                    CURRENT_CMD_VEL.signal(cmd_vel);
                }
            }
        }
    }
}
