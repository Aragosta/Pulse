//! Quadrature encoder counted in PIO, so the count needs no interrupt and no hand-written state in the loop.
//!
//! The program is Raspberry Pi's pico-examples `quadrature_encoder.pio` (BSD-3-Clause, Copyright (c) 2021 Raspberry
//! Pi (Trading) Ltd.). A 16-entry jump table indexed by (previous AB, current AB) steps the count kept in Y; writing
//! anything non-zero to the TX FIFO asks for the count, which arrives on the RX FIFO within 18 PIO cycles. It must be
//! loaded at address 0 (`mov pc, isr` jumps into the table by address) with the input shift direction set to left.
//! `examples/single_joint/tests/encoder_pio.rs` runs this exact assembled program in an emulator.

pub fn program() -> pio::Program<32> {
    pio::pio_asm!(
        ".origin 0",
        "    jmp update",    // 00 -> 00
        "    jmp decrement", // 00 -> 01
        "    jmp increment", // 00 -> 10
        "    jmp update",    // 00 -> 11
        "    jmp increment", // 01 -> 00
        "    jmp update",    // 01 -> 01
        "    jmp update",    // 01 -> 10
        "    jmp decrement", // 01 -> 11
        "    jmp decrement", // 10 -> 00
        "    jmp update",    // 10 -> 01
        "    jmp update",    // 10 -> 10
        "    jmp increment", // 10 -> 11
        "    jmp update",    // 11 -> 00
        "    jmp increment", // 11 -> 01
        "decrement:",
        "    jmp y--, update", // 11 -> 10
        ".wrap_target",
        "update:",
        "    set x, 0",
        "    pull noblock",
        "    mov x, osr",
        "    mov osr, isr",
        "    jmp !x, sample_pins",
        "    mov isr, y",
        "    push",
        "sample_pins:",
        "    mov isr, null",
        "    in osr, 2",
        "    in pins, 2",
        "    mov pc, isr",
        "increment:",
        "    mov y, ~y",
        "    jmp y--, increment_cont",
        "increment_cont:",
        "    mov y, ~y",
        ".wrap",
    )
    .program
}
