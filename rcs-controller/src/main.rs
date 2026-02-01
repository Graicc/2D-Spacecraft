#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use crate::{
    control::control_task,
    messaging::{messaging_task, CURRENT_IMU_READING},
    wifi::UdpConnection,
};
use defmt::{debug, println};
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_net::{udp::UdpSocket, DhcpConfig, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::{assign_resources, clock::CpuClock};
use esp_radio::wifi::{ClientConfig, ModeConfig};
use {esp_backtrace as _, esp_println as _};

mod control;
mod messaging;
mod wifi;

assign_resources! {
    Resources<'d> {
        wifi: WifiResources<'d> {
            wifi: WIFI,
        }
    }
}

// From https://github.com/esp-rs/esp-hal/blob/main/examples/wifi/embassy_access_point_with_sta/src/main.rs
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0);

    let r = split_resources!(p);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    spawner
        .spawn(messaging_task(p.UART0, p.GPIO21, p.GPIO22))
        .unwrap();

    spawner
        .spawn(control_task(
            p.LEDC, p.GPIO14, p.GPIO27, p.GPIO26, p.GPIO25, p.GPIO33, p.GPIO32,
        ))
        .unwrap();

    {
        let esp_radio_ctrl = mk_static!(esp_radio::Controller, esp_radio::init().unwrap());

        let (mut controller, interfaces) =
            esp_radio::wifi::new(esp_radio_ctrl, r.wifi.wifi, Default::default()).unwrap();

        const SSID: &str = "craig";
        const PASSWORD: &str = "tacholycos";

        let wifi_sta_device = interfaces.sta;
        let mut dhcp_config: DhcpConfig = Default::default();
        let hostname: heapless::String<32> = heapless::String::try_from("ESP32").unwrap();
        dhcp_config.hostname = Some(hostname);
        let sta_config = embassy_net::Config::dhcpv4(dhcp_config);

        let rng = esp_hal::rng::Rng::new();
        let seed = (rng.random() as u64) << 32 | rng.random() as u64;

        let (sta_stack, sta_runner) = embassy_net::new(
            wifi_sta_device,
            sta_config,
            mk_static!(StackResources<4>, StackResources::<4>::new()),
            seed,
        );

        let station_config = ModeConfig::Client(
            ClientConfig::default()
                .with_ssid(SSID.into())
                .with_password(PASSWORD.into()),
        );
        controller.set_config(&station_config).unwrap();

        spawner.spawn(wifi::connection(controller)).unwrap();
        spawner.spawn(wifi::net_task(sta_runner)).unwrap();

        let sta_address = loop {
            if let Some(config) = sta_stack.config_v4() {
                let address = config.address.address();
                println!("Got IP: {}", address.octets());
                break address;
            }
            println!("Waiting for IP...");
            Timer::after(Duration::from_millis(500)).await;
        };

        println!(
            "connect to the ap `{}` and point your browser to http://{}:8080/",
            SSID,
            sta_address.octets()
        );

        let mut rx_buffer: [u8; _] = [0; 4096];
        let mut tx_buffer: [u8; _] = [0; 4096];
        let mut rx_meta = [embassy_net::udp::PacketMetadata::EMPTY; 16];
        let mut tx_meta = [embassy_net::udp::PacketMetadata::EMPTY; 16];
        let mut socket = UdpSocket::new(
            sta_stack,
            &mut rx_meta,
            &mut rx_buffer,
            &mut tx_meta,
            &mut tx_buffer,
        );
        socket.bind((sta_address, 1234)).unwrap();

        let mut udp_connection = UdpConnection::new(socket);

        println!("Waiting for first packet to identify host");
        let data = messages::receive_single_read::<messages::WifiMessage, _>(&mut udp_connection)
            .await
            .unwrap();
        println!("Got first packet");

        loop {
            match select(
                messages::receive_single_read::<messages::WifiMessage, _>(&mut udp_connection),
                CURRENT_IMU_READING.wait(),
            )
            .await
            {
                embassy_futures::select::Either::First(Ok(messages::WifiMessage::Ping(num))) => {
                    debug!("Got ping, sending pong");
                    messages::send(&mut udp_connection, &messages::WifiMessage::Pong(num))
                        .await
                        .unwrap();
                }
                embassy_futures::select::Either::First(Ok(
                    messages::WifiMessage::Pong(_) | messages::WifiMessage::IMUReading(_),
                )) => {
                    todo!()
                }
                embassy_futures::select::Either::First(Err(_)) => {}
                embassy_futures::select::Either::Second(imu_reading) => {
                    debug!("Sending IMU reading");
                    messages::send(
                        &mut udp_connection,
                        &messages::WifiMessage::IMUReading(imu_reading),
                    )
                    .await
                    .unwrap();
                }
            }
        }
    }
}

mod tests {}
