use embassy_stm32::{time::hz, timer::simple_pwm::SimplePwmChannels};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

const PWM_DENOM: u32 = 2083;
const PWM_ZERO: u32 = 1000;
const PWM_BASE_ON: u32 = 2000;
const PWM_FAN_ON: u32 = 1337;

pub static SET_MOTOR_STATE: Signal<CriticalSectionRawMutex, messages::MotorValues> = Signal::new();

fn u8_to_fan_pwm(val: u8) -> u32 {
    PWM_ZERO + (val as u32 * (PWM_FAN_ON - PWM_ZERO) / (u8::MAX as u32))
}

fn u8_to_base_pwm(val: u8) -> u32 {
    PWM_ZERO + (val as u32 * (PWM_BASE_ON - PWM_ZERO) / (u8::MAX as u32))
}

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
    let pwm_b = embassy_stm32::timer::simple_pwm::SimplePwm::new(
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
    let SimplePwmChannels {
        ch1: _,
        ch2: _,
        ch3: mut base_motor,
        ch4: mut right_motor,
    } = pwm_b.split();

    left_motor.enable();
    right_motor.enable();
    base_motor.enable();

    left_motor.set_duty_cycle_fraction(PWM_ZERO, PWM_DENOM);
    right_motor.set_duty_cycle_fraction(PWM_ZERO, PWM_DENOM);
    base_motor.set_duty_cycle_fraction(PWM_ZERO, PWM_DENOM);

    loop {
        let motor_values = SET_MOTOR_STATE.wait().await;

        left_motor.set_duty_cycle_fraction(u8_to_fan_pwm(motor_values.left), PWM_DENOM);
        right_motor.set_duty_cycle_fraction(u8_to_fan_pwm(motor_values.right), PWM_DENOM);
        base_motor.set_duty_cycle_fraction(u8_to_base_pwm(motor_values.base), PWM_DENOM);
    }
}
