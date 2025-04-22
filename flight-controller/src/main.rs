#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    time::hz,
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led = Output::new(p.PB5, Level::High, Speed::Low);
    // let mut led2 = Output::new(p.PB0, Level::High, Speed::Low);

    let pwm_pin = embassy_stm32::timer::simple_pwm::PwmPin::new_ch3(
        p.PB0,
        embassy_stm32::gpio::OutputType::PushPull,
    );
    let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(
        p.TIM3,
        None,
        None,
        Some(pwm_pin),
        None,
        // hz(16_000),
        hz(480),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let mut motor = pwm.ch3();
    motor.enable();

    loop {
        led.set_high();
        // off
        motor.set_duty_cycle_fraction(1000, 2083);
        Timer::after_secs(5).await;

        // on
        // motor.set_duty_cycle_fraction(1337, 2083);
        motor.set_duty_cycle_fraction(1000, 2083);
        led.set_low();
        Timer::after_secs(2).await;
    }
}
