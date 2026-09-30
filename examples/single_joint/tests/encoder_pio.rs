//! The RP2350 encoder program (`examples/single_joint_rp2350/src/encoder.rs`), assembled exactly as it is flashed,
//! run in a minimal PIO emulator against simulated A/B pins. The emulator implements only the instructions the
//! program uses, with the RP2350 datasheet semantics that matter here: `jmp y--` tests before decrementing, a
//! non-blocking `pull` on an empty TX FIFO copies X into OSR, `in` shifts left (as the firmware configures it), and
//! execution wraps from `.wrap` to `.wrap_target`. Anything else panics, so the emulator cannot silently diverge.

#[path = "../../single_joint_rp2350/src/encoder.rs"]
mod encoder;

use std::collections::VecDeque;

struct Sm {
    code: Vec<u16>,
    wrap: (u8, u8),
    pc: u8,
    x: u32,
    y: u32,
    isr: u32,
    osr: u32,
    tx: VecDeque<u32>,
    rx: VecDeque<u32>,
    /// Bit 0 = phase A (in_pin_base), bit 1 = phase B.
    pins: u32,
}

impl Sm {
    fn new() -> Sm {
        let p = encoder::program();
        assert_eq!(p.origin, Some(0), "the jump table needs address 0");
        Sm {
            code: p.code.to_vec(),
            wrap: (p.wrap.source, p.wrap.target),
            pc: 0, // the HAL starts a state machine with `jmp <offset>`
            x: 0,
            y: 0,
            isr: 0,
            osr: 0,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            pins: 0,
        }
    }

    fn step(&mut self) {
        let i = self.code[self.pc as usize];
        assert_eq!((i >> 8) & 0x1f, 0, "no delay or side-set expected");
        let (op, a, idx) = (i >> 13, (i >> 5) & 7, (i & 0x1f) as u32);
        let src = |sm: &Sm, s: u16| match s {
            0 => sm.pins,
            1 => sm.x,
            2 => sm.y,
            3 => 0,
            6 => sm.isr,
            7 => sm.osr,
            _ => panic!("source {s}"),
        };
        let mut jump = None;
        match op {
            0 => {
                let take = match a {
                    0 => true,
                    1 => self.x == 0,
                    3 => self.y == 0,
                    4 => {
                        let t = self.y != 0;
                        self.y = self.y.wrapping_sub(1);
                        t
                    }
                    _ => panic!("jmp condition {a}"),
                };
                if take {
                    jump = Some(idx as u8);
                }
            }
            2 => {
                assert!((1..32).contains(&idx), "in bit count {idx}");
                self.isr = (self.isr << idx) | (src(self, a) & ((1 << idx) - 1));
            }
            4 if i & 0x80 == 0 => {
                assert!(self.rx.len() < 4, "RX FIFO full: push would stall");
                self.rx.push_back(self.isr);
                self.isr = 0;
            }
            4 => match self.tx.pop_front() {
                Some(v) => self.osr = v,
                None => {
                    assert_eq!(i & 0x20, 0, "blocking pull on an empty FIFO");
                    self.osr = self.x;
                }
            },
            5 => {
                let s = src(self, i & 7);
                let v = match (i >> 3) & 3 {
                    0 => s,
                    1 => !s,
                    o => panic!("mov op {o}"),
                };
                match a {
                    1 => self.x = v,
                    2 => self.y = v,
                    5 => jump = Some(v as u8),
                    6 => self.isr = v,
                    7 => self.osr = v,
                    d => panic!("mov destination {d}"),
                }
            }
            7 => match a {
                1 => self.x = idx,
                2 => self.y = idx,
                d => panic!("set destination {d}"),
            },
            _ => panic!("opcode {op}"),
        }
        self.pc = match jump {
            Some(t) => t,
            None if self.pc == self.wrap.0 => self.wrap.1,
            None => self.pc + 1,
        };
    }

    /// Hold the pins for as long as the slowest loop iteration (14 instructions) and then some.
    fn set(&mut self, ab: u32) {
        self.pins = ab;
        for _ in 0..20 {
            self.step();
        }
    }

    /// What the firmware does each tick: ask, then wait for the answer (at most 18 cycles).
    fn count(&mut self) -> i32 {
        self.tx.push_back(1);
        for _ in 0..18 {
            self.step();
            if let Some(c) = self.rx.pop_front() {
                return c as i32;
            }
        }
        panic!("no count within 18 cycles");
    }
}

/// Gray-code order with A leading: 00 -> 01 -> 11 -> 10.
const FWD: [u32; 4] = [0b01, 0b11, 0b10, 0b00];

#[test]
fn counts_every_edge_and_returns_to_zero() {
    let mut sm = Sm::new();
    sm.set(0b00);
    assert_eq!(sm.count(), 0);
    for (k, ab) in FWD.iter().cycle().take(400).enumerate() {
        sm.set(*ab);
        if k % 37 == 0 {
            // Reading mid-motion must neither lose nor add a step.
            assert_eq!(sm.count().abs(), k as i32 + 1);
        }
    }
    let forward = sm.count();
    assert_eq!(forward.abs(), 400);
    // Back the same way: through zero and 400 below it (two's complement survives the u32 FIFO).
    for ab in FWD.iter().rev().cycle().skip(1).take(800) {
        sm.set(*ab);
    }
    assert_eq!(sm.count(), -forward);
}

#[test]
fn a_skipped_state_is_not_a_step() {
    let mut sm = Sm::new();
    sm.set(0b00);
    // 00<->11 and 01<->10 change both phases at once: direction unknown, so no count.
    for ab in [0b11, 0b00, 0b11, 0b00] {
        sm.set(ab);
    }
    assert_eq!(sm.count(), 0);
    sm.set(0b01);
    assert_eq!(sm.count().abs(), 1, "a single real edge still counts");
    for ab in [0b10, 0b01, 0b10, 0b01] {
        sm.set(ab);
    }
    assert_eq!(sm.count().abs(), 1);
}

#[test]
fn powering_up_between_states_counts_nothing() {
    // Pull-ups idle both phases high; the program starts assuming 00, and 00 -> 11 must not count.
    let mut sm = Sm::new();
    sm.set(0b11);
    assert_eq!(sm.count(), 0);
}
