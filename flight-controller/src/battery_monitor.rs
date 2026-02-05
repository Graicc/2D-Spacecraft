use cortex_m::singleton;
use embassy_stm32::adc::{AdcChannel, RegularConversionMode, RingBufferedAdc, SampleTime};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};
use log::warn;

use crate::BatteryMonitorResource;

pub static LOW_BATTERY: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new_with(false);
pub static BATTERY_VOLTAGE: Signal<CriticalSectionRawMutex, u32> = Signal::new();

const CELL_COUNT: u32 = 4;
/// In mV
const CELL_LOW_VOLTAGE_CUTOFF: u32 = 3500;
const LOW_VOLTAGE_CUTOFF: u32 = CELL_COUNT * CELL_LOW_VOLTAGE_CUTOFF;

// Ignore voltage readings under this (because the lipo isn't plugged in) in mV
const MIN_VOLTAGE: u32 = 100;

/// Converts adc mv (adjusted based on vreint sample) to battery voltage
fn convert_to_battery_millivolts(mv: u32) -> u32 {
    11 * mv + 12
}

#[embassy_executor::task]
pub async fn battery_monitor_task(res: BatteryMonitorResource) {
    const ADC_BUF_SIZE: usize = 1024;
    let adc_data: &mut [u16; ADC_BUF_SIZE] =
        singleton!(ADCDAT : [u16; ADC_BUF_SIZE] = [0u16; ADC_BUF_SIZE]).unwrap();

    let mut adc = embassy_stm32::adc::Adc::new(res.adc);
    let mut vrefint = adc.enable_vrefint();
    let vrefint_sample = adc.blocking_read(&mut vrefint, embassy_stm32::adc::SampleTime::CYCLES112);

    let convert_to_millivolts = |sample| {
        // From http://www.st.com/resource/en/datasheet/DM00071990.pdf
        // 6.3.24 Reference voltage
        const VREFINT_MV: u32 = 1210; // mV

        u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)
    };

    let mut adc: RingBufferedAdc<embassy_stm32::peripherals::ADC1> = adc.into_ring_buffered(
        res.dma,
        adc_data,
        [(res.pin.degrade_adc(), SampleTime::CYCLES112)].into_iter(),
        RegularConversionMode::Continuous,
    );

    let mut buffer = [0u16; ADC_BUF_SIZE / 2];
    adc.start();

    let mut ticker = Ticker::every(Duration::from_secs(1));

    let mut low_voltage_count = 0;

    // TODO: this stopped working
    loop {
        match adc.read(&mut buffer).await {
            Ok(count) => {
                const TARGET_COUNT: usize = 10;
                if count < TARGET_COUNT {
                    warn!("< 10 adc measurements?");
                    continue;
                }

                let voltage = buffer
                    .iter()
                    .take(TARGET_COUNT)
                    .map(|&sample| convert_to_battery_millivolts(convert_to_millivolts(sample)))
                    .sum::<u32>()
                    / (TARGET_COUNT as u32);

                if voltage > MIN_VOLTAGE && voltage < LOW_VOLTAGE_CUTOFF {
                    low_voltage_count += 1;
                    warn!("Low battery voltage");
                }

                if low_voltage_count > 5 {
                    warn!("Battery dead");
                    // TODO: this value is never used
                    LOW_BATTERY.sender().send(true);
                }

                BATTERY_VOLTAGE.signal(voltage);
            }
            Err(_e) => {
                // warn!("Error: {:?}", e);
                warn!("Error in ADC");
                buffer = [0u16; ADC_BUF_SIZE / 2];
                adc.start();
            }
        }

        ticker.next().await;
    }
}
