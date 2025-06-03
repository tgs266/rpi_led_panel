use std::{thread::sleep, time::Duration};

use crate::{
    gpio_bits,
    registers::{ClkRegisters, GPIOFunction, GPIORegisters, PWMRegisters, TimeRegisters},
};

const PWM_BASE_TIME_NS: u32 = 2;
const EMPIRICAL_NANOSLEEP_OVERHEAD_US: u32 = 12;
const MINIMUM_NANOSLEEP_TIME_US: u32 = 5;

struct Pulse {
    start_time: u64,
    sleep_hint_us: u32,
}

pub(crate) trait PinPulser {
    fn send_pulse(
        &mut self,
        bitplane: usize,
        pwm_registers: &mut PWMRegisters,
        time_registers: &mut TimeRegisters,
    );
    fn wait_pulse_finished(
        &mut self,
        time_registers: &mut TimeRegisters,
        pwm_registers: &mut PWMRegisters,
    );
}

pub(crate) struct HardwarePinPulser {
    sleep_hints_us: Vec<u32>,
    pulse_periods: Vec<u32>,
    current_pulse: Option<Pulse>,
}

pub(crate) struct TimerBasedPinPulser {
    sleep_hints_ns: Vec<u64>,
}

impl TimerBasedPinPulser {
    pub fn new(bitplane_timings_ns: &[u32]) -> Self {
        let sleep_hints_ns = bitplane_timings_ns
            .iter()
            .map(|&t| t.saturating_sub(EMPIRICAL_NANOSLEEP_OVERHEAD_US * 1000) as u64)
            .collect();
        TimerBasedPinPulser { sleep_hints_ns }
    }

    fn sleep_precise(&self, time_registers: &TimeRegisters, duration_ns: u64) {
        let min_total_ns = (EMPIRICAL_NANOSLEEP_OVERHEAD_US + MINIMUM_NANOSLEEP_TIME_US) * 1000;

        if duration_ns > min_total_ns.into() {
            let sleep_ns = duration_ns - ((EMPIRICAL_NANOSLEEP_OVERHEAD_US as u64) * 1000);
            sleep(Duration::from_nanos(sleep_ns));
        }

        let start_us = time_registers.get_time();
        let target_us = start_us + duration_ns / 1000;
        while time_registers.get_time() < target_us {
            std::hint::spin_loop();
        }
    }
}

impl PinPulser for TimerBasedPinPulser {
    fn send_pulse(
        &mut self,
        bitplane: usize,
        _pwm_registers: &mut PWMRegisters,
        time_registers: &mut TimeRegisters,
    ) {
        let ns = self.sleep_hints_ns[bitplane];
        self.sleep_precise(time_registers, ns);
    }

    fn wait_pulse_finished(
        &mut self,
        _time_registers: &mut TimeRegisters,
        _pwm_registers: &mut PWMRegisters,
    ) {
        // No-op for timer-based pulse
    }
}

impl HardwarePinPulser {
    pub(crate) fn new(
        pins: u32,
        bitplane_timings_ns: &[u32],
        pwm_registers: &mut PWMRegisters,
        gpio_registers: &mut GPIORegisters,
        clk_registers: &mut ClkRegisters,
    ) -> Self {
        let sleep_hints_us = bitplane_timings_ns.iter().map(|t| t / 1000).collect();

        let time_base = bitplane_timings_ns[0];

        if pins == gpio_bits!(18) {
            gpio_registers.select_function(18, GPIOFunction::Alt5);
        } else if pins == gpio_bits!(12) {
            gpio_registers.select_function(12, GPIOFunction::Alt0);
        } else {
            unreachable!()
        }

        pwm_registers.reset_pwm();
        clk_registers.init_pwm_divider((time_base / 2) / PWM_BASE_TIME_NS);

        let pulse_periods = bitplane_timings_ns
            .iter()
            .map(|timing| 2 * timing / time_base)
            .collect();

        Self {
            sleep_hints_us,
            pulse_periods,
            current_pulse: None,
        }
    }
}

impl PinPulser for HardwarePinPulser {
    fn send_pulse(
        &mut self,
        bitplane: usize,
        pwm_registers: &mut PWMRegisters,
        time_registers: &mut TimeRegisters,
    ) {
        if self.pulse_periods[bitplane] < 16 {
            pwm_registers.set_pwm_pulse_period(self.pulse_periods[bitplane]);
            pwm_registers.push_fifo(self.pulse_periods[bitplane]);
        } else {
            let period_fraction = self.pulse_periods[bitplane] / 8;
            pwm_registers.set_pwm_pulse_period(period_fraction);
            for _ in 0..8 {
                pwm_registers.push_fifo(period_fraction);
            }
        }

        pwm_registers.push_fifo(0);
        pwm_registers.push_fifo(0);

        self.current_pulse = Some(Pulse {
            start_time: time_registers.get_time(),
            sleep_hint_us: self.sleep_hints_us[bitplane],
        });

        pwm_registers.enable_pwm();
    }

    fn wait_pulse_finished(
        &mut self,
        time_registers: &mut TimeRegisters,
        pwm_registers: &mut PWMRegisters,
    ) {
        let Some(pulse) = self.current_pulse.take() else {
            return;
        };

        let already_elapsed_us = time_registers.get_time() - pulse.start_time;
        let remaining_time_us = u64::from(pulse.sleep_hint_us).saturating_sub(already_elapsed_us);
        time_registers.sleep_at_most(remaining_time_us);

        while !pwm_registers.fifo_empty() {
            std::thread::yield_now();
        }

        pwm_registers.reset_pwm();
    }
}
