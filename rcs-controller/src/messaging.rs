use embassy_futures::select;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};
use esp_hal::peripherals::{GPIO21, GPIO22, UART0};
use messages::{Message, Receiver};

pub static CURRENT_CMD_VEL: Signal<CriticalSectionRawMutex, messages::CMDVelValues> = Signal::new();
pub static CURRENT_STATE: Signal<CriticalSectionRawMutex, messages::StateUpdate> = Signal::new();

pub static MESSAGES_TO_SEND: Channel<CriticalSectionRawMutex, messages::Message, 16> =
    Channel::new();

#[embassy_executor::task]
pub async fn messaging_task(uart: UART0<'static>, rx: GPIO21<'static>, tx: GPIO22<'static>) {
    let cfg = esp_hal::uart::Config::default().with_baudrate(115_200);
    let mut uart = esp_hal::uart::Uart::new(uart, cfg)
        .unwrap()
        .with_rx(rx)
        .with_tx(tx)
        .into_async();

    let mut receiver: Receiver<Message, 256, 32> = messages::Receiver::new();

    loop {
        match select::select(receiver.receive(&mut uart), MESSAGES_TO_SEND.receive()).await {
            select::Either::First(Ok(Message::CMDVel(cmd_vel))) => {
                // println!(
                //     "Got current vel command: vx={} vy={} vt={}",
                //     cmd_vel.vx, cmd_vel.vy, cmd_vel.vt
                // );
                CURRENT_CMD_VEL.signal(cmd_vel);
            }
            select::Either::First(Ok(Message::StateUpdate(state_update))) => {
                // println!("Got imu reading",);
                CURRENT_STATE.signal(state_update);
            }
            select::Either::First(Ok(Message::MotorValues(_) | Message::HeartBeat)) => todo!(), // Should never be received
            select::Either::First(Err(_)) => {}
            select::Either::Second(message) => {
                messages::send(&mut uart, &message).await.unwrap();
            }
        }
    }
}
