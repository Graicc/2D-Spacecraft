use embassy_stm32::time::hz;
use embassy_time::Timer;

const PWM_DENOM: u32 = 2083;
const PWM_ZERO: u32 = 1000;
const PWM_ONE: u32 = 1337;

#[embassy_executor::task]
pub async fn motor_task(motor: crate::MotorResource) {
    let motor_1_pin = embassy_stm32::timer::simple_pwm::PwmPin::new(
        motor.motor_1_pin,
        embassy_stm32::gpio::OutputType::PushPull,
    );
    let motor_2_pin = embassy_stm32::timer::simple_pwm::PwmPin::new(
        motor.motor_2_pin,
        embassy_stm32::gpio::OutputType::PushPull,
    );
    let motor_3_pin = embassy_stm32::timer::simple_pwm::PwmPin::new(
        motor.motor_3_pin,
        embassy_stm32::gpio::OutputType::PushPull,
    );
    let motor_4_pin = embassy_stm32::timer::simple_pwm::PwmPin::new(
        motor.motor_4_pin,
        embassy_stm32::gpio::OutputType::PushPull,
    );
    let mut pwm_a = embassy_stm32::timer::simple_pwm::SimplePwm::new(
        motor.tim3,
        None,
        None,
        Some(motor_1_pin),
        Some(motor_2_pin),
        // hz(16_000),
        hz(480),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let mut pwm_b = embassy_stm32::timer::simple_pwm::SimplePwm::new(
        motor.tim2,
        None,
        None,
        Some(motor_4_pin),
        Some(motor_3_pin),
        // hz(16_000),
        hz(480),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let mut left_motor = pwm_a.ch3();
    let mut right_motor = pwm_b.ch4();
    let mut base_motor = pwm_b.ch3();

    // TODO: actually do the motors
    loop {
        Timer::after_secs(5).await;
    }

    base_motor.enable();

    loop {
        // Off
        base_motor.set_duty_cycle_fraction(PWM_ZERO, PWM_DENOM);
        Timer::after_secs(5).await;

        // On
        base_motor.set_duty_cycle_fraction(PWM_ONE, PWM_DENOM);
        Timer::after_secs(5).await;
    }
}
