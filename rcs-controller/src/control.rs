use defmt::println;
use embassy_time::{Duration, Ticker};
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::Ledc;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, LowSpeed};
use esp_hal::peripherals::{GPIO14, GPIO25, GPIO26, GPIO27, GPIO32, GPIO33, LEDC};
use esp_hal::time::Rate;
use {esp_backtrace as _, esp_println as _};

use crate::messaging;

fn cmd_vel_to_fans(cmd_vel: messages::CMDVelValues) -> nalgebra::Vector6<f32> {
    #[rustfmt::skip]
    let local_xyt_to_rcs = nalgebra::Matrix6x3::from_row_slice(&[
        0.0, -0.5, 1.0,
        0.0, -0.5, -1.0,
        -1.0, 0.0, 0.0,
        0.0, 0.5, 1.0,
        0.0, 0.5, -1.0,
        1.0, 0.0, 0.0,
    ]);

    local_xyt_to_rcs * nalgebra::Vector3::new(cmd_vel.vx, cmd_vel.vy, cmd_vel.vt)
}

#[embassy_executor::task]
pub async fn control_task(
    ledc: LEDC<'static>,
    p1: GPIO14<'static>,
    p2: GPIO27<'static>,
    p3: GPIO26<'static>,
    p4: GPIO25<'static>,
    p5: GPIO33<'static>,
    p6: GPIO32<'static>,
) {
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .unwrap();

    let mut fans = [
        ledc.channel(channel::Number::Channel0, p1),
        ledc.channel(channel::Number::Channel0, p2),
        ledc.channel(channel::Number::Channel0, p3),
        ledc.channel(channel::Number::Channel0, p4),
        ledc.channel(channel::Number::Channel0, p5),
        ledc.channel(channel::Number::Channel0, p6),
    ];
    for fan in &mut fans {
        fan.configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    }

    for fan in &mut fans {
        fan.set_duty(0).unwrap();
    }

    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        let cmd_vel = messaging::CURRENT_CMD_VEL.wait().await;

        println!("Setting fans!");
        let fan_speeds = cmd_vel_to_fans(cmd_vel);
        let fan_values = fan_speeds
            .iter()
            .map(|value| f32::clamp(value * 100.0 / 10.0, 0.0, 100.0) as u8);

        for (fan, value) in fans.iter_mut().zip(fan_values) {
            match fan.set_duty(value) {
                Ok(_) => (),
                Err(e) => println!("Error setting fan value to {}: {}", value, e),
            }
        }

        ticker.next().await;
    }
}
