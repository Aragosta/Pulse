//! The single joint on a Raspberry Pi Pico 2 (RP2350): every 125 us, read the sensors, run the generated
//! `Firmware::step`, drive the H-bridge. Nothing here decides anything: conversions and contract enforcement are
//! `pulse_joint::io` (host-tested over every ADC code), behaviour is the generated step. Wiring and calibration are at
//! the top of `pulse_joint::io`.
//!
//! Build: `cargo build --release` in this directory (the target is set in `.cargo/config.toml`).
//! Flash:  `cargo run --release` with the board in BOOTSEL mode (needs `picotool`), or `probe-rs run --chip RP235x`.
//! Observe: `STEP_CYCLES_MAX` (longest step, in 150 MHz cycles) and `OVERRUNS` (ticks that ended past their
//! deadline) are plain symbols a debugger or probe can read while it runs. They are measurements, not a WCET proof.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};
use cortex_m::peripheral::DWT;
use cortex_m::register::fpscr::{self, RMode};
use embedded_hal::digital::{OutputPin, PinState};
use embedded_hal::pwm::SetDutyCycle;
use hal::Clock;
use hal::fugit::ExtU32;
use hal::gpio::{FunctionPio0, PullUp};
use hal::pio::{PIOExt, PinDir, ShiftDirection};
use panic_halt as _;
use pulse_joint::{generated::Firmware, io, params::BASE_HZ};
use rp235x_hal as hal;

mod encoder;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_HZ: u32 = 12_000_000;
/// `io::PWM_TOP` is computed for this clock; boot refuses any other.
const SYS_HZ: u32 = 150_000_000;
const TICK_US: u64 = 1_000_000 / BASE_HZ as u64;
/// Encoder phases A and B on consecutive pins (the PIO program reads them as one 2-bit value).
const ENC_A: u8 = 2;
const FAULT: f32 = pulse_joint::FAULT as f32;

#[unsafe(no_mangle)]
pub static STEP_CYCLES_MAX: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static OVERRUNS: AtomicU32 = AtomicU32::new(0);

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // SEMANTICS.md §7 assumes IEEE binary32, round-to-nearest, no flush-to-zero. Check it instead of trusting it:
    // a panic here halts before the motor is ever driven (PWM outputs are still plain GPIO, low).
    let f = fpscr::read();
    assert!(
        !f.fz() && f.rmode() == RMode::Nearest,
        "FPU mode differs from the proofs"
    );

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    assert_eq!(clocks.system_clock.freq().to_Hz(), SYS_HZ);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // H-bridge: PWM slice 0, IN1 = GPIO16 (channel A), IN2 = GPIO17 (channel B). Compare values start at 0 (coast).
    let mut slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let bridge = &mut slices.pwm0;
    bridge.set_div_int(1);
    bridge.set_top(io::PWM_TOP);
    bridge.enable();
    bridge.channel_a.output_to(pins.gpio16);
    bridge.channel_b.output_to(pins.gpio17);

    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut current = hal::adc::AdcPin::new(pins.gpio26).unwrap();
    let mut temp = hal::adc::AdcPin::new(pins.gpio27).unwrap();
    let mut command = hal::adc::AdcPin::new(pins.gpio28).unwrap();

    // Encoder phases on GPIO2/3, pulled up, read by the PIO program in `encoder.rs`.
    let _enc = (
        pins.gpio2
            .into_function::<FunctionPio0>()
            .into_pull_type::<PullUp>(),
        pins.gpio3
            .into_function::<FunctionPio0>()
            .into_pull_type::<PullUp>(),
    );
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&encoder::program()).unwrap();
    let (mut sm, mut rx, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed)
        .in_pin_base(ENC_A)
        .in_shift_direction(ShiftDirection::Left)
        .clock_divisor_fixed_point(1, 0)
        .build(sm0);
    sm.set_pindirs([(ENC_A, PinDir::Input), (ENC_A + 1, PinDir::Input)]);
    sm.start();

    let mut led = pins.gpio25.into_push_pull_output();
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    // A hung loop must not leave the bridge driven: missing 80 ticks resets the chip, which boots with PWM low.
    watchdog.pause_on_debug(true);
    watchdog.start(10_000.micros());

    let mut fw = Firmware::new();
    let mut next = timer.get_counter().ticks() + TICK_US;
    loop {
        while timer.get_counter().ticks() < next {}
        next += TICK_US;
        let t0 = DWT::cycle_count();

        tx.write(1);
        let count = loop {
            if let Some(c) = rx.read() {
                break c as i32;
            }
        };
        // A failed conversion is a bad sample (NaN), never a made-up value.
        let amps = adc.read(&mut current).map_or(f32::NAN, io::current_amps);
        let celsius = adc.read(&mut temp).map_or(f32::NAN, io::temp_c);
        let target = adc.read(&mut command).map_or(f32::NAN, io::command_rad);

        let (volts, state, _setpoint) = fw.step(io::theta_rad(count), amps, celsius, target);

        let [in1, in2] = io::pwm(volts);
        let _ = bridge.channel_a.set_duty_cycle(in1);
        let _ = bridge.channel_b.set_duty_cycle(in2);
        let _ = led.set_state(PinState::from(state == FAULT));
        watchdog.feed();

        STEP_CYCLES_MAX.fetch_max(DWT::cycle_count().wrapping_sub(t0), Relaxed);
        if timer.get_counter().ticks() > next {
            OVERRUNS.fetch_add(1, Relaxed);
        }
    }
}

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 4] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(
        c"Pulse single joint: generated firmware step at 8 kHz"
    ),
    // The IR the step was generated from: `picotool info -a` on a board names the evidence that covers it.
    hal::binary_info::str!(
        hal::binary_info::consts::TAG_RASPBERRY_PI,
        hal::binary_info::consts::ID_RP_PROGRAM_FEATURE,
        pulse_joint::generated::IR_HASH
    ),
];
