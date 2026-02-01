use embassy_stm32::{
    gpio::{Level, Output, Speed},
    spi::Spi,
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Duration, Ticker};
use messages::IMUReading;
use mpu6000::MPU6000;

use crate::{IMUResource, state::State};

pub static CURRENT_STATE: Signal<CriticalSectionRawMutex, State> = Signal::new();
pub static CURRENT_IMU: Signal<CriticalSectionRawMutex, IMUReading> = Signal::new();

#[embassy_executor::task]
pub async fn imu_task(res: IMUResource) {
    let IMUResource {
        peri,
        sck,
        mosi,
        miso,
        cs,
    } = res;

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(1_000_000);

    let accel_sens = mpu6000::registers::AccelerometerSensitive::Sensitive2048;
    let gyro_sens = mpu6000::registers::GyroSensitive::Sensitive16_4;

    let spi1 = Spi::new_blocking(peri, sck, mosi, miso, spi_config);
    let spi_bus = mpu6000::bus::SpiBus::new(spi1, Output::new(cs, Level::Low, Speed::Low), Delay);
    let mut mpu6000 = MPU6000::new(spi_bus);
    mpu6000
        .reset(&mut Delay)
        .unwrap_or_else(|_| panic!("Failed to reset MPU6000"));
    mpu6000
        .set_sleep(false)
        .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"));

    mpu6000
        .set_accelerometer_sensitive(accel_sens)
        .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"));
    mpu6000
        .set_gyro_sensitive(gyro_sens)
        .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"));

    let dt = Duration::from_millis(10);
    let dt_s: f32 = 0.001 * 10.0;
    let mut ticker = Ticker::every(dt);

    let mut state: State = Default::default();
    loop {
        let acc = mpu6000
            .read_acceleration()
            .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"))
            / accel_sens;

        let gyro = mpu6000
            .read_gyro()
            .unwrap_or_else(|_| panic!("Failed to initialize MPU6000"))
            / gyro_sens;
        log::info!(
            "{},{},{},{},{},{}",
            acc[0],
            acc[1],
            acc[2],
            gyro[0],
            gyro[1],
            gyro[2]
        );

        let ax_local = acc[0];
        let ay_local = acc[1];
        let at = gyro[2];

        let ax = ax_local * libm::cosf(state.pt) - ay_local * libm::sinf(state.pt);
        let ay = ax_local * libm::sinf(state.pt) + ay_local * libm::cosf(state.pt);

        state.vx += ax * dt_s;
        state.vy += ay * dt_s;
        state.vt += at * dt_s;

        state.px += state.vx * dt_s;
        state.py += state.vy * dt_s;
        state.pt += state.vt * dt_s;

        CURRENT_STATE.signal(state.clone());
        CURRENT_IMU.signal(IMUReading { acc, gyro });

        ticker.next().await;
    }
}
