use std::{thread::sleep, time::Duration};

use crate::{
    gpio_bits,
    registers::{ClkRegisters, GPIOFunction, GPIORegisters, PWMRegisters, TimeRegisters},
};

const PWM_BASE_TIME_NS: u32 = 2;
const EMPIRICAL_NANOSLEEP_OVERHEAD_US: u32 = 12;
const MINIMUM_NANOSLEEP_TIME_US: u32 = 5;

/// Simple struct to hold pulse timing info (for hardware pulser).
struct Pulse {
    start_time: u64,
    sleep_hint_us: u32,
}

/// Abstracts pulse timing (manual or hardware).
pub(crate) trait PinPulser {
    fn send_pulse(
        &mut self,
        bitplane: usize,
        gpio_registers: &mut GPIORegisters,
        pwm_registers: &mut PWMRegisters,
        time_registers: &mut TimeRegisters,
    );
    fn wait_pulse_finished(
        &mut self,
        time_registers: &mut TimeRegisters,
        pwm_registers: &mut PWMRegisters,
    );
}

/// Software-timed, manual GPIO pulser: toggles pin low, waits, toggles high.
pub(crate) struct TimerBasedPinPulser {
    sleep_hints_ns: Vec<u32>,
    pins: u32,
}

impl TimerBasedPinPulser {
    pub fn new(bitplane_timings_us: &[u32], pins: u32) -> Self {
        let sleep_hints_ns = bitplane_timings_us
            .iter()
            .map(|&t| t / 1000)
            .collect();
        Self { sleep_hints_ns, pins }
    }    
}

impl PinPulser for TimerBasedPinPulser {
    fn send_pulse(
        &mut self,
        bitplane: usize,
        gpio_registers: &mut GPIORegisters,
        _pwm_registers: &mut PWMRegisters,
        time_registers: &mut TimeRegisters,
    ) {
        let us = self.sleep_hints_ns[bitplane];
        // Exactly like C++: drive pin(s) low, wait, drive high
        gpio_registers.write_clr_bits(self.pins);
        sleep(Duration::from_nanos(us as u64));
        gpio_registers.write_set_bits(self.pins);
    }

    fn wait_pulse_finished(
        &mut self,
        _time_registers: &mut TimeRegisters,
        _pwm_registers: &mut PWMRegisters,
    ) {
        // No-op for timer-based; pulse is atomic.
    }
}

/// Hardware PWM pin pulser: loads pulse length into FIFO, lets hardware strobe pin.
pub(crate) struct HardwarePinPulser {
    sleep_hints_us: Vec<u32>,
    pulse_periods: Vec<u32>,
    current_pulse: Option<Pulse>,
    pins: u32,
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

        // Set correct alternate function for hardware PWM pin.
        if pins == gpio_bits!(18) {
            gpio_registers.select_function(18, GPIOFunction::Alt5);
        } else if pins == gpio_bits!(12) {
            gpio_registers.select_function(12, GPIOFunction::Alt0);
        } else {
            unreachable!("Hardware PWM can only use GPIO 12 or 18");
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
            pins,
        }
    }
}

impl PinPulser for HardwarePinPulser {
    fn send_pulse(
        &mut self,
        bitplane: usize,
        _gpio_registers: &mut GPIORegisters,
        pwm_registers: &mut PWMRegisters,
        time_registers: &mut TimeRegisters,
    ) {
        // Just like C++: push pulse periods to FIFO
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
        let Some(pulse) = self.current_pulse.take() else { return; };
        let already_elapsed_us = time_registers.get_time() - pulse.start_time;
        let remaining_time_us = u64::from(pulse.sleep_hint_us).saturating_sub(already_elapsed_us);
        time_registers.sleep_at_most(remaining_time_us);

        while !pwm_registers.fifo_empty() {
            std::thread::yield_now();
        }
        pwm_registers.reset_pwm();
    }
}
