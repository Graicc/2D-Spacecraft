#![no_std]
#![no_main]

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_futures::select::Either::{First, Second};
use embassy_futures::select::select;
use embassy_stm32::{Peri, usb};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals,
    time::Hertz,
};
use embassy_time::{Duration, Instant, Ticker};

use crate::battery_monitor::BATTERY_VOLTAGE;
use crate::imu::{CURRENT_IMU, CURRENT_STATE};
use crate::logger::logger_task;
use crate::motor::SET_MOTOR_STATE;
use crate::state::State;

use messages::{self, Message};

mod battery_monitor;
mod imu;
mod logger;
mod motor;
mod state;

assign_resources! {
    logger: LoggerResource {
        peri: USB_OTG_FS,
        dp: PA12,
        dm: PA11,
    },
    imu: IMUResource {
        peri: SPI1,
        sck: PA5,
        mosi: PA7,
        miso: PA6,
        cs: PA4
    },
    motor: MotorResource {
        // Motor 1: Left prop
        // Motor 2: Unused
        // Motor 3: Right prop
        // Motor 4: Base
        motor_1_pin: PB0,
        motor_2_pin: PB1,
        motor_3_pin: PA3,
        motor_4_pin: PA2,
        tim2: TIM2,
        tim3: TIM3,
    },
    battery_monitor: BatteryMonitorResource {
        adc: ADC1,
        pin: PC2,
        dma: DMA2_CH0
    }
}

embassy_stm32::bind_interrupts!(struct SerialPort1IRQs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
});

embassy_stm32::bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let config = {
        let mut config = embassy_stm32::Config::default();
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
        config
    };

    let p = embassy_stm32::init(config);
    let r = split_resources!(p);

    {
        const EP_OUT_BUFFER_SIZE: usize = 10 * 1024;
        pub static EP_OUT_BUFFER: static_cell::StaticCell<[u8; EP_OUT_BUFFER_SIZE]> =
            static_cell::StaticCell::new();
        let mut config = embassy_stm32::usb::Config::default();
        config.vbus_detection = false;
        let usb_driver = embassy_stm32::usb::Driver::new_fs(
            r.logger.peri,
            Irqs,
            r.logger.dp,
            r.logger.dm,
            EP_OUT_BUFFER.init([0; _]),
            config,
        );
        spawner.spawn(logger_task(usb_driver)).unwrap();
    }

    spawner.spawn(imu::imu_task(r.imu)).unwrap();

    spawner.spawn(motor::motor_task(r.motor)).unwrap();

    spawner
        .spawn(battery_monitor::battery_monitor_task(r.battery_monitor))
        .unwrap();

    // Onboard led
    let mut led: Output<'_> = Output::new(p.PB5, Level::High, Speed::Low);
    led.set_low();

    let mut uart_config = embassy_stm32::usart::Config::default();
    uart_config.baudrate = 115_200;
    let uart = embassy_stm32::usart::Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        SerialPort1IRQs,
        p.DMA2_CH7,
        p.DMA2_CH2,
        uart_config,
    )
    .unwrap();
    let (mut uart_tx, uart_rx) = uart.split();
    let mut uart_rx_buf = [0u8; 1024];
    let mut uart_rx = uart_rx.into_ring_buffered(&mut uart_rx_buf);

    let mut receiver: messages::Receiver<Message, 256, 32> = messages::Receiver::new();

    let mut last_heartbeat = Instant::now();

    let mut last_voltage: u32 = 0;
    let mut current_motor_values: messages::MotorValues = Default::default();

    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        let state = CURRENT_STATE.wait().await;

        let target_state: State = Default::default();

        let err = target_state - state;

        // Just doing a P controller for now
        let cmd_vel = messages::CMDVelValues {
            vx: err.px * 0.1,
            vy: err.py * 0.1,
            vt: err.pt * 0.1,
        };
        let message = messages::Message::CMDVel(cmd_vel);

        match messages::send(&mut uart_tx, &message).await {
            Ok(_) => {}
            Err(_) => log::error!("Failed to send message"),
        }

        last_voltage = BATTERY_VOLTAGE.try_take().unwrap_or(last_voltage);

        if let Some(imu_status) = CURRENT_IMU.try_take() {
            match messages::send(
                &mut uart_tx,
                &messages::Message::StateUpdate(messages::StateUpdate {
                    imu_status,
                    voltage: last_voltage,
                    motor_values: current_motor_values.clone(),
                }),
            )
            .await
            {
                Ok(_) => {}
                Err(_) => log::error!("Failed to send IMU"),
            }
        }

        match select(receiver.receive(&mut uart_rx), ticker.next()).await {
            First(Ok(Message::HeartBeat)) => last_heartbeat = Instant::now(),
            First(Ok(Message::MotorValues(motor_values))) => {
                log::info!("Motor values: {:?}", &motor_values);
                current_motor_values = motor_values;
                SET_MOTOR_STATE.signal(current_motor_values.clone());
            }
            First(Ok(Message::CMDVel(_) | Message::StateUpdate(_))) => todo!(),
            First(Err(_)) => todo!(),
            Second(_) => {}
        }

        if (Instant::now() - last_heartbeat).as_secs() > 2 {
            log::error!("Heartbeat time out!");
            current_motor_values = Default::default();
            SET_MOTOR_STATE.signal(Default::default());
            led.set_high(); // Show error
        }
    }
}
