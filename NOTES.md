# Pulse: decisions and paths forward

Running log. Newest decisions at the top of each section. Status is one of: **Proposed**, **Decided**, **Superseded**.
Facts are marked **verified** (checked against a source, linked) or **unverified** (from memory or a secondary source; check before relying on it).

## Decisions

### D-006: What the IR is for, and its shape (Proposed, 2026-09-29)

**Role.** The IR is the one contract between layers: frontends (Python, Rumoca/Modelica, FMI import) write it; checks (Class 1/2/3), codegen, the WCET provider and the MCP server read it. Everything downstream is generated from it, including `copperconfig.ron`; once that is generated, the drift test in `examples/single_joint/src/ir.rs` goes away.

**Prior art (checked 2026-09-29).**
- **Rumoca** has Parsed -> Flattened -> DAE -> Solver IRs. Its DAE IR tags variables (state, algebraic, discrete, parameter) and equations (continuous, discrete update, event condition), is a superset of Base Modelica, and can be reduced to index 1 ([arXiv 2606.14998](https://arxiv.org/html/2606.14998v1)). Continuous blocks should **reference Rumoca's DAE IR, not reinvent it**.
- **FMI 3.0 clocks**: time-based periodic clocks (interval + shift) and triggered clocks, model partitions activated per clock ([spec](https://fmi-standard.org/docs/3.0/)). Vocabulary for rates beyond integer Hz.
- **Lingua Franca**: logical time tags, deterministic ordering at equal tags, deadlines tying logical to physical time ([ACM](https://dl.acm.org/doi/fullHtml/10.1145/3448128)). The semantics Class 1 already assumes; the IR should state it.
- **Lustre / Vélus**: a clock calculus type-checks every cross-rate use; Vélus proves compilation correct in Coq ([velus.inria.fr](https://velus.inria.fr/)). Our hold rules are a hand-rolled clock calculus; treat them as a type system.
- **AADL** end-to-end flow latency over periodic threads and ports: our staleness check, generalised to paths.
- **MCP (2025-11-25)**: tools declare `inputSchema`/`outputSchema` and return `structuredContent`; resources by URI ([spec](https://modelcontextprotocol.io/specification/2025-11-25/server/tools)). The OpenModelica MCP proposal ([issue](https://github.com/OpenModelica/OpenModelica/issues/15385)) shows the loop agents want: generate, compiler validates, revise. Only works with structured violations.

**What the IR must carry, per class.**

| Need | For | Status |
|---|---|---|
| Serialized, versioned (JSON), stable ids, source spans | frontends, MCP, evidence | **done**: `IR_VERSION` (now 3), serde with `deny_unknown_fields`, `span` |
| Block -> implementation binding (`imp`) and edge payload type (`msg`) | codegen; seed of opaque blocks and typed ports | **done (v2)** |
| Stable violation codes (`C1-HOLD-MISSING`, ...) | CLI, CI, MCP | **done** |
| Well-formedness at the trust boundary: version, unique identifier ids, `imp`/`msg` are Rust paths (`Ir::validate`, `IR-*` codes) | every pass; frontends and agents are untrusted | **done (v3)** |
| Declared delays (`delay_ticks`, Modelica `previous`); every feedback loop needs one (`C1-LOOP`) | Class 1 causality | **done (v3)** |
| Content hash that WCET results and proofs bind to | Class 1/3 evidence | later |
| Clocks: rational period + shift; triggered clocks | Class 1 | later (integer Hz today) |
| WCET: budget + slot for provider bound with provenance | Class 1 (D-001) | budget only |
| Latency declared on paths, not single edges | Class 1 | later |
| Typed ports: dtype, unit, range | Class 2/3 | later |
| State machines as data: states, guards, transitions | Class 2 (exhaustive, reachable, deterministic) | later; FSM is opaque Rust today |
| Small total expression language (no loops/alloc) for updates, guards, contracts | Class 3 (SMT) + codegen read the same terms | later |
| Opaque blocks (hand-written Rust, policy/agent blocks) checked only at contract boundary, labelled reduced assurance; policy blocks add output envelope + monitor + fallback | Class 3, runtime assurance | later |

**MCP.** A thin server over `pulse-ir`, never a second implementation. Resources `pulse://ir`, `pulse://report`, `pulse://schema`; tools `validate`, `explain` (how a bound was derived), `apply_patch` (JSON Patch RFC 6902 -> new report + diff), `simulate`, later `generate`. Agent edits are untrusted input and pass the same checks; agents edit IR or frontend source, never generated code.

**Done in this step.** `pulse-ir` types are serde-serializable with owned ids, `Ir.version`, optional `span` on blocks/edges; `class1::Violation` has a stable `code`; `Report` serializes. `single-joint --ir-json` prints the IR; a test round-trips it through JSON and re-checks Class 1.

**Done in step 2.** `pulse_ir::copper::emit` generates `copperconfig.ron` from the IR (tasks from blocks with `imp`, connections in IR edge order, holds as comments). The config is a committed golden file because Copper's macro takes a literal path relative to the crate root, not `OUT_DIR`. `copper_config_is_generated` fails on a stale or hand-edited file; regenerate with `PULSE_BLESS=1 cargo test -p single_joint`. The old id/edge drift test is gone. Task glue (`tasks.rs`) is still hand-written.

### D-001: How we get a WCET bound (Proposed, 2026-09-29)

**Why it matters.** Class 1 (Temporal Determinism) claims "WCET <= budget on every run". Today the example only *observes* task times on a macOS host, which is noise, not a bound. The WCET method also constrains which chip we can prove anything on, so the two are decided together.

**Constraint found: static tool support decides the target.**

| Core | aiT (static) | Notes |
|---|---|---|
| Cortex-M0, M0+ (via M0), M1, M3 | Yes (verified) | e.g. STM32F103, LM3S811 |
| Cortex-M4 / M4F | Yes (verified) | Infineon XMC4500, Microchip ATSAME51 named |
| Cortex-R4F / R5F | Yes (verified) | TI TMS570 |
| Cortex-M7, Cortex-A53 | Only via TimeWeaver (verified) | Hybrid, trace-based; needs trace hardware |
| **Cortex-M33 (RP2350)** | **Not listed (verified absent)** | Armv8-M instructions may be supported; a timing model for the core is not listed |
| RISC-V (RP2350 Hazard3) | Not listed (verified absent) | |

Source: <https://www.absint.com/ait/arm.htm>

**Options**

| # | Method | Gives a proof? | Fits Pulse? | Main risk |
|---|---|---|---|---|
| A | Static, binary-level, commercial (AbsInt aiT) | Yes, if the hardware model is right | Best fit: analyzes the exact shipped binary, language-agnostic; SCADE + aiT is the industry precedent for model -> generated code -> WCET | Commercial license (price unverified); supported cores are M4-and-older |
| B | Static, open-source framework (OTAWA) | Yes, if the model is right | Binary-level, adaptable | Core support for Cortex-M unverified (could not reach the project site); a validation study found about one third of tested opcode/operand combinations wrong in the AURIX TC275 model, so models need validating against hardware ([paper](https://drops.dagstuhl.de/storage/01oasics/oasics-vol072-wcet2019/OASIcs.WCET.2019.6/OASIcs.WCET.2019.6.pdf)) |
| C | Static, LLVM-integrated (LLVMTA, Saarland) | Yes, if the model is right | Weak: it analyzes code from its own patched LLVM, so the analyzed binary is not the one we deploy unless we ship its output | Patched LLVM, Linux, research-grade; last commit 2025-09-02 (verified); ARM and RISC-V generic models per its 2022 paper; Rust never mentioned |
| D | Hybrid, trace-based (TimeWeaver, RapiTime) | Tighter bound, evidence from measured traces | Only for M7/A53-class parts | Needs ETM-style trace hardware |
| E | Measurement + stated margin (cycle counter high-water mark) | No | Works on any chip, including RP2350 | Not safe: only sees the worst case we happened to hit |
| F | Predictability by construction (SRAM execution, one core, no cache, single-path kernels, compiler-known loop bounds) | Not alone; makes A/B/E far tighter | Yes, and Pulse's generator knows every loop bound | Needs discipline in generated code |
| G | Probabilistic (extreme-value statistics) | Statistical | No | Only sound on randomized-timing hardware |

**Proposed decision (layered, tool-agnostic):**

1. **Interface first.** `wcet_budget_ns` stays the contract in the IR. A pluggable provider supplies a per-block bound (from a binary + hardware model) and Class 1 checks `bound <= budget`. No tool is hard-wired.
2. **Two tracks, because the cheap chip and the provable chip differ:**
   - **Track B (develop and measure): RP2350.** Copper supports it bare-metal, it is cheap, and it gives real non-host timing. Run hot code from SRAM on one core, read the cycle counter, report as **measured, not proven**.
   - **Track A (prove): a Cortex-M4F part** that a static tool covers (aiT names Infineon XMC4500 and Microchip ATSAME51). Analyze the shipped binary with aiT (or OTAWA if its M4 support checks out).
3. **Discipline in generated code:** bounded loops with compiler-emitted flow facts, no panics, no recursion, no dynamic dispatch, no allocation. Pin `rustc`, `panic=abort`, `codegen-units=1`; rerun the analysis in CI on every build (a WCET result is only valid for one exact binary + hardware config + tool version).
4. **Cross-check as a test oracle:** require `measured <= static <= budget`. A measurement above the static bound means the hardware model is wrong.
5. **LLVMTA is not a dependency.** Revisit only if we control the codegen path so analyzed == deployed.

**Resolved 2026-09-29:**
- aiT offers a **free 30-day trial** on your own application (signed Evaluation License Agreement, [AbsInt](https://www.absint.com/ait/contact.htm), verified). Price of a full license is still unknown.
- The RP2350's Cortex-M33 has a usable DWT cycle counter (`DWT->CYCCNT`); people use it on RP2350 per the Raspberry Pi forums (secondary source, confirm on the first board).

**Open questions (need an answer before Track A starts):**- Does Copper's bare-metal support cover an M4F part (SAME51 / XMC4500)? README lists RP2350 as bare-metal and an STM32H7 (Cortex-M7) flight-controller example; M4F support is **unverified** and may need a board-support port.
- Does OTAWA support Cortex-M4? **Unverified.**
- RP2350 SRAM timing: 520 KB in ten concurrently accessible banks ([Wikipedia](https://en.wikipedia.org/wiki/RP2350)); "single-cycle SRAM, no cache" comes from a secondary source, **verify in the datasheet**. XIP flash has a cache (2 x 16 KB per the datasheet), which is why hot code should run from SRAM.

**Next actions:** (1) request the aiT trial (needs a compiled M4F binary to be worth the 30 days, so do it after Track A hardware is chosen); (2) confirm Copper on an M4F board or scope the port; (3) RP2350 port with cycle-counter probes (see D-005); (4) add the provider interface to `pulse-ir` when the first real bound exists.

### D-005: RP2350 port stages (In progress, 2026-09-29)

Copper's reference bare-metal platform is a **Pimoroni Pico Plus 2 (RP2350B)** with a **CMSIS-DAP debug probe** (e.g. WeAct MiniDebugger, DAPLink version) and `probe-rs`; example `examples/cu_rp2350_skeleton`, run with `cargo run-arm`; logging needs an SD card ([Copper bare-metal guide](https://copper-project.github.io/copper-rs/Baremetal-Development/), verified). Hardware needed for Stage 2: the board, the probe, a few jumper wires.

- **Stage 1: shared `no_std` core (done).** `crates/pulse-joint` holds the sensor model, both controllers and the thermal FSM with no Copper/std/alloc. The host example now calls it; regression oracle (derating at tick 16840, fault at tick 37160) is unchanged. It builds for `thumbv8m.main-none-eabihf`.
- **Stage 2: firmware (needs hardware).** Copper tasks on the board, plant simulated in f32 on the MCU (processor-in-the-loop, no analog hardware), DWT cycle probes around only the controller calls, run hot code from SRAM, report over probe-rs/RTT.
- **Stage 3: bound.** Measured per-block worst cycles vs budgets; compare against the static bound once Track A exists.

**Lessons so far**
- `f32::clamp` panics on a NaN bound and drags in float-formatting code, but bare `max`/`min` is not the fix: `f32::max(NaN, x) == x`, so a NaN setpoint came out as full reverse current and a NaN scale disabled the limit. Generated code guards with comparisons that NaN fails (`if x >= 0.0`, `is_finite`) and maps NaN to the safe value (0 A, `FAULT`).
- **Check for panics on the final firmware binary, not the library.** rustc treats small functions as cross-crate-inlinable and does not emit them in the library, so a library-level grep can miss them.

### D-002: Multi-rate on Copper via decimation (Decided, 2026-09-29)

Copper has one loop rate (`runtime.rate_target_hz`, verified). Pulse runs an 8 kHz base tick; 200 Hz blocks act on every 40th tick and republish their held output between activations. Every cross-rate edge must declare its hold in the IR (`Decimate(n)` fast to slow, `Zoh` slow to fast); `pulse-ir` rejects any edge that doesn't. Tick budget is checked as the sum of WCET budgets on the busiest tick, because the whole graph runs sequentially in one slot.

### D-003: Sensor dropout needs a declared bound (Decided, 2026-09-29)

Probabilistic dropout has no deterministic staleness bound. `SensorSpec` carries `max_dropout_run`, and the derived worst-case age is `(latency + max_dropout_run + decimation + 1)` ticks.

### D-004: Python frontend, file-based IR handoff (Proposed, deferred)

Python is a compile-time authoring layer only, never linked into the binary (Python has a GIL and an allocator, which would void WCET and `no_std`). It emits a versioned IR file that the Rust compiler reads; Rumoca/Modelica is a second frontend into the same IR. PyO3 only if the file loop proves too slow. Not started: the IR shape must settle on the single-joint example first.

## Paths forward (backlog)

- [ ] Resolve D-001 open questions (aiT full-license price, Copper on M4F, OTAWA M4 support, RP2350 SRAM timing)
- [x] D-005 Stage 1: shared `no_std` core
- [ ] D-005 Stage 2: RP2350 firmware with cycle-counter probes (needs the board + probe)
- [ ] CI check: no panic references in the final firmware binary
- [ ] WCET provider interface in `pulse-ir` once a real bound exists
- [x] D-006 step 1: serializable, versioned IR with source spans and stable violation codes
- [x] Generate `copperconfig.ron` from the IR; drift test replaced by a golden-file test
- [x] IR well-formedness, declared delays and zero-delay-loop check; NaN-safe controllers; `pulse-joint` tests; stall oracle (16840/37160) as a test
- [ ] Budget model: 200 Hz tasks run (republish) on every tick, but Class 1 counts them only on firing ticks
- [ ] Differential test: random IRs, brute-force tick simulation, observed ages/budgets <= what `class1` derives
- [ ] CI: `cargo test`, clippy, `thumbv8m.main-none-eabihf` build
- [ ] Align hold vocabulary with Modelica 3.3 synchronous (`subSample`, `hold`, `previous`) and add ModelingToolkit.jl clocks to prior art
- [ ] Minimal Python frontend that writes the single-joint IR JSON (D-004)
- [ ] Generate task glue from the IR (needs FSM as data / expression language)
- [ ] IR: typed ports, FSM as data, expression language (prerequisites for Class 2/3)
- [ ] JSON Schema for the IR (`schemars`) and the MCP server, when a frontend or agent consumes the file
- [ ] Add "Frontend" section to `README.md` (D-004)
- [ ] Structural Coverage and Symbolic Proof on the single-joint example (only after Class 1 is solid, per the README)
- [ ] Agent/policy blocks with WCET budgets (last, per the README)
