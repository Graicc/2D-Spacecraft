use core::fmt::Display;

use defmt::println;
use embassy_net::{udp::UdpSocket, IpEndpoint, Runner};
use embassy_time::{Duration, Timer};
use embedded_io_async::{ErrorKind, ErrorType};
use esp_radio::wifi::{WifiController, WifiDevice, WifiEvent};

pub struct UdpConnection<'a> {
    endpoint: Option<IpEndpoint>,
    socket: UdpSocket<'a>,
}

impl<'a> UdpConnection<'a> {
    pub fn new(socket: UdpSocket<'a>) -> Self {
        Self {
            endpoint: None,
            socket,
        }
    }
}

#[derive(Debug)]
pub struct UdpConnectionError();

impl Display for UdpConnectionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("UdpConnectionError")
    }
}

impl core::error::Error for UdpConnectionError {}

impl embedded_io_async::Error for UdpConnectionError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Unsupported
    }
}

impl<'a> ErrorType for UdpConnection<'a> {
    type Error = UdpConnectionError;
}

impl<'a> embedded_io_async::Write for UdpConnection<'a> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        match self.endpoint {
            Some(endpoint) => self
                .socket
                .send_to(buf, endpoint)
                .await
                .map(|_| buf.len())
                .map_err(|_| UdpConnectionError()),
            None => Err(UdpConnectionError()),
        }
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<'a> embedded_io_async::Read for UdpConnection<'a> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let (size, metadata) = self
            .socket
            .recv_from(buf)
            .await
            .map_err(|_| UdpConnectionError())?;

        if self.endpoint.is_none() {
            self.endpoint = Some(metadata.endpoint);
        }

        Ok(size)
    }
}

// From https://github.com/esp-rs/esp-hal/blob/main/examples/wifi/embassy_access_point_with_sta/src/main.rs
#[embassy_executor::task]
pub async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");

    println!("Starting wifi");
    controller.start_async().await.unwrap();
    println!("Wifi started!");

    loop {
        if matches!(controller.is_started(), Ok(true)) {
            println!("About to connect...");

            match controller.connect_async().await {
                Ok(_) => {
                    // wait until we're no longer connected
                    controller.wait_for_event(WifiEvent::StaDisconnected).await;
                    println!("Station disconnected");
                }
                Err(_e) => {
                    println!("Failed to connect to wifi");
                    Timer::after(Duration::from_millis(5000)).await
                }
            }
        } else {
            return;
        }
    }
}

// From https://github.com/esp-rs/esp-hal/blob/main/examples/wifi/embassy_access_point_with_sta/src/main.rs
#[embassy_executor::task(pool_size = 2)]
pub async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
