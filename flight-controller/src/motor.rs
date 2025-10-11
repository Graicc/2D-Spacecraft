use embassy_stm32::time::hz;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn motor_task(motor: crate::MotorResource) {
    let motor_pin = embassy_stm32::timer::simple_pwm::PwmPin::new(
        motor.motor_pin,
        embassy_stm32::gpio::OutputType::PushPull,
    );
    let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(
        motor.tim,
        None,
        None,
        Some(motor_pin),
        None,
        // hz(16_000),
        hz(480),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let mut motor = pwm.ch3();

    motor.enable();

    loop {
        // Off
        motor.set_duty_cycle_fraction(1000, 2083);
        Timer::after_secs(5).await;

        // On
        motor.set_duty_cycle_fraction(1337, 2083);
        Timer::after_secs(5).await;
    }
}
