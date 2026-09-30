# Pulse IR semantics

What an IR program means. Every other part of Pulse is checked against this: the timing proofs (Class 1), the invariant and range proofs (Class 3), the reference interpreter (`Compute::step` on `graph::firmware`), and the generated firmware. Where the code and this page disagree, one of them is a bug.

## 1. The world and the firmware

An IR is a set of **blocks** and **edges** at one **base rate** (`base_rate_hz`, a whole number of nanoseconds per tick).

- A block **with** `compute` is **firmware**: it is generated, verified and flashed.
- A block **without** `compute` is the **outside world**: plant, sensor, command source, actuator, telemetry. Pulse does not generate or verify it; it only states contracts about it.
- An edge connects an output port to an input port. An edge from the outside world into firmware is a **firmware input** named `{block}__{port}`; an edge from firmware to the outside world is a **firmware output**. Every firmware input port is driven by exactly one edge.

The firmware is one function, `step(inputs) -> outputs`, called once per base tick.

## 2. Time

Ticks are numbered `k = 0, 1, 2, …`. A block with rate `r` has period `p = base / r` ticks (`r` must divide the base rate) and **fires** on ticks where `k mod p = 0`.

On every tick, every firmware block runs, in an order where every zero-delay producer runs before its consumers (a zero-delay cycle is rejected). A block that does not fire this tick **holds**: its state and outputs keep their values from its last firing. So on every tick every block costs time, which is why the tick budget is the sum of all budgets.

A consumer always reads its producer's **current output**, the value after the producer ran this tick (or held). A cross-rate edge must declare how that read is legal: `Decimate(n)` for fast → slow, `Zoh` for slow → fast. The declaration changes no behaviour; it is checked by Class 1. An edge with `delay_ticks = d` delivers the value from `d` ticks earlier. Today delays are allowed only on edges into the outside world (e.g. the plant integrating the voltage held from the previous tick); a delay into firmware is refused until the generated step has delay buffers.

## 3. What a block computes

A block's `compute` has input ports, named **params** (constants), **state** (with an initial literal), ordered **defs**, output ports, and **next** (new state values; unlisted state keeps its value). One firing:

1. Evaluate the defs in order. Each may read inputs, params, state and earlier defs.
2. The outputs are the named defs, inputs or state (their values before step 3).
3. All next-state values are computed from the old state, then assigned together.

**Expressions** are total and have no loops, calls or side effects. Numbers are IEEE-754 **binary32** (f32), round-to-nearest-even, with subnormals (no flush-to-zero):

- `+ − × ÷` and negation are the IEEE operations; `÷ 0` gives ±∞ or NaN, never a trap.
- `max`/`min` are Rust's `f32::max`/`min`: if one operand is NaN the other is returned.
- Comparisons are IEEE (every comparison with NaN is false); `is_finite` is false for NaN and ±∞.
- `select(c, a, b)` is `a` if `c` else `b`. Both sides are pure, so which one is evaluated does not matter.
- Nothing is rewritten: `0 × x` is NaN when `x` is, so it stays. Evaluation order is the tree's order.

The same expressions read as f64 are the **model**; read as f32 they are the **firmware**. The difference between the two is rounding, and is measured, not assumed away.

## 4. Components

A **component** is a named `compute` in the IR's library. `Use { name, component, bind }` places an instance: each component input is bound to an expression in the enclosing scope. **Flattening** inlines it: its params, state and defs appear as `{name}__{x}`, an input bound to a variable or literal is substituted where it is used, and any other binding becomes a def. The enclosing scope may read only the instance's outputs. Names written by a user never contain `__`; that separator belongs to flattening. Every analysis and codegen works on the flat form.

## 4a. State machines

`Stmt::Fsm` declares a state variable, named states (each state's code is its index, and its name is a param with that code), an initial state, and transitions in priority order, each with the states it leaves from (empty: any) and a boolean guard. On each firing the first transition whose `from` matches the current state and whose guard holds is taken; if none is, the state stays. So every machine is deterministic and total by construction. The machine is lowered to a state variable (range `[0, n-1]`, proved as an invariant) and a `select` chain before any analysis. Structural checks: states are unique, every referenced state exists, and every state is reachable from the initial one over the transition graph. Whether a guard can ever hold is not checked yet.

## 5. Units

Every numeric port, param and state may carry a unit (`V`, `A`, `V/(A*s)`, `1` for dimensionless). `+ − max min select` and comparisons need equal units, `× ÷` combine them, a bare literal adopts its context. Symbols are independent: no conversions, so one spelling per quantity. Components are checked once; each binding against the component's declared input unit; each firmware edge between blocks must connect equal units.

## 6. What the proofs mean

- **Contracts.** An input port's `range` is an assumption: its samples are finite and inside it. With `glitch`, a good sample is inside the range and a bad one may be NaN or ±∞. Without a range, a sample may be anything.
- **Ranges are on f32 values**, so a declared range `[lo, hi]` means its f32 hull (the smallest f32 interval containing it).
- **State invariants** (`C3-INVARIANT`): each ranged state variable is inside its range initially, and one tick taken from inside every range, with any input the contracts allow, lands inside again. By induction it holds on every tick.
- **Output ranges** (`C3-RANGE`): under those invariants, each ranged output (firmware outputs and every block output) is inside its range and never NaN, on every tick.
- **Components are proved once** (assume-guarantee): each component's invariants and output ranges are proved on their own, assuming only its input contracts. Every instance must then feed each contracted input a value that meets the contract (`C3-CONTRACT`: never NaN and inside the range; with `glitch`, its good samples inside the range), so the component's own proof applies to that instance. The whole firmware is also proved as one, where inputs between blocks carry what their producers are proved to output.
- **Timing** (Class 1): rate ratios, holds on every cross-rate read, a worst-case age on every edge, no zero-delay loops, and the sum of all WCET budgets within one tick. That each block meets its budget is **not** proved until a WCET bound exists (NOTES.md D-001).

## 7. What is trusted, not proved

- The outside world meets its contracts (sensor ranges, glitch behaviour, command range, sensor latency and dropout bounds).
- The target's FPU runs in IEEE binary32 mode: round-to-nearest, no flush-to-zero.
- `rustc` compiles the generated Rust as written. The generated code is the reference walk printed as Rust, and is tested bit-identical to the interpreter; it is not proved.
- Blocks meet their WCET budgets.

`pulse_ir::evidence` lists these for a specific IR, with the IR's content hash, so every claim names the exact model it was proved for.
