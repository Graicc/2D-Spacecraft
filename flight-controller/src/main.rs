#![no_std]
#![no_main]

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_stm32::Peri;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals,
    time::Hertz,
};
use embassy_time::{Duration, Ticker};

use crate::imu::CURRENT_STATE;
use crate::state::State;

use messages;

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
        motor_pin: PB0,
        tim: TIM3,
    }
}

embassy_stm32::bind_interrupts!(struct SerialPort1IRQs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
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

    spawner.spawn(logger::logger_task(r.logger)).unwrap();

    spawner.spawn(imu::imu_task(r.imu)).unwrap();

    spawner.spawn(motor::motor_task(r.motor)).unwrap();

    let mut cfg = embassy_stm32::usart::Config::default();
    cfg.baudrate = 115_200;
    let mut uart = embassy_stm32::usart::Uart::new(
        p.USART1,
        p.PA10,
        p.PA9,
        SerialPort1IRQs,
        p.DMA2_CH7,
        p.DMA2_CH2,
        cfg,
    )
    .unwrap();

    // Onboard led
    let mut led: Output<'_> = Output::new(p.PB5, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        led.toggle();

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

        match messages::send(&mut uart, &message).await {
            Ok(_) => {}
            Err(_) => log::error!("Failed to send message"),
        }

        ticker.next().await;
    }
}
