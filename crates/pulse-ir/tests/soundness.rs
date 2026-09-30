//! Soundness of the Class 3 interval proof over whole blocks: random computes (arithmetic, comparisons, logic,
//! `is_finite`, `select` with the branch refinement `assume` does, state) are run concretely in f32 on samples the
//! contracts allow, and every output and next-state value must lie inside what `step_intervals` derives. The per-op
//! test in `class3` covers arithmetic alone; this covers how the proof composes.

use pulse_ir::class3::{Bv, Iv, step_intervals};
use pulse_ir::expr::{Compute, Def, Expr, F32, Port, StateVar, Stmt, Ty, Val, num, select, var};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        // xorshift64*
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
    fn pick<T: Copy>(&mut self, xs: &[T]) -> T {
        xs[self.below(xs.len())]
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

const LITS: [f64; 11] = [
    0.0, -0.0, 1.0, -1.0, 0.1, -3.5, 3.5, 24.0, 1e30, -1e30, 1e-30,
];
const WILD: [f32; 8] = [
    f32::NAN,
    f32::INFINITY,
    f32::NEG_INFINITY,
    f32::MAX,
    -f32::MAX,
    0.0,
    -0.0,
    1e-40,
];

struct Case {
    c: Compute,
    /// How to sample each input (and each state variable): its range, and whether NaN/inf may appear instead.
    ins: Vec<(Option<[f64; 2]>, bool)>,
}

fn range(r: &mut Rng) -> [f64; 2] {
    let (a, b) = (r.pick(&LITS), r.pick(&LITS));
    [a.min(b), a.max(b)]
}

fn num_expr(r: &mut Rng, scope: &[String], depth: u32) -> Expr {
    if depth == 0 || r.below(4) == 0 {
        return if r.below(3) == 0 {
            num(r.pick(&LITS))
        } else {
            var(&scope[r.below(scope.len())])
        };
    }
    let a = num_expr(r, scope, depth - 1);
    let b = num_expr(r, scope, depth - 1);
    match r.below(8) {
        0 => a + b,
        1 => a - b,
        2 => a * b,
        3 => a / b,
        4 => a.max(b),
        5 => a.min(b),
        6 => -a,
        _ => select(bool_expr(r, scope, depth - 1), a, b),
    }
}

fn bool_expr(r: &mut Rng, scope: &[String], depth: u32) -> Expr {
    // The first two shapes are the ones `assume` refines on; the rest exercise cmp/logic on their own.
    let x = var(&scope[r.below(scope.len())]);
    let c = num(r.pick(&LITS));
    match r.below(if depth == 0 { 3 } else { 7 }) {
        0 => match r.below(4) {
            0 => x.lt(c),
            1 => x.le(c),
            2 => x.gt(c),
            _ => x.ge(c),
        },
        1 => x.is_finite(),
        2 => x.eq(c),
        3 => num_expr(r, scope, depth - 1).lt(num_expr(r, scope, depth - 1)),
        4 => bool_expr(r, scope, depth - 1).and(bool_expr(r, scope, depth - 1)),
        5 => select(
            bool_expr(r, scope, depth - 1),
            bool_expr(r, scope, depth - 1),
            bool_expr(r, scope, depth - 1),
        ),
        _ => bool_expr(r, scope, depth - 1)
            .or(bool_expr(r, scope, depth - 1))
            .not(),
    }
}

fn port(name: &str, ty: Ty, range: Option<[f64; 2]>, glitch: bool) -> Port {
    Port {
        name: name.into(),
        ty,
        unit: None,
        range,
        glitch,
    }
}

fn random_case(r: &mut Rng) -> Case {
    let mut ins = Vec::new();
    let mut inputs = Vec::new();
    for i in 0..1 + r.below(3) {
        let (rg, glitch) = match r.below(3) {
            0 => (None, true),
            1 => (Some(range(r)), false),
            _ => (Some(range(r)), true),
        };
        inputs.push(port(&format!("i{i}"), Ty::F32, rg, rg.is_some() && glitch));
        ins.push((rg, glitch));
    }
    let mut state = Vec::new();
    for s in 0..r.below(3) {
        let rg = (r.below(2) == 0).then(|| range(r));
        state.push(StateVar {
            name: format!("s{s}"),
            init: num(0.0),
            unit: None,
            range: rg,
        });
        ins.push((rg, rg.is_none()));
    }
    let mut scope: Vec<String> = inputs.iter().map(|p| p.name.clone()).collect();
    scope.extend(state.iter().map(|s| s.name.clone()));
    let mut defs = Vec::new();
    let mut outputs = Vec::new();
    for d in 0..1 + r.below(5) {
        let name = format!("d{d}");
        let (expr, ty) = if r.below(5) == 0 {
            (bool_expr(r, &scope, 2), Ty::Bool)
        } else {
            (num_expr(r, &scope, 3), Ty::F32)
        };
        defs.push(Stmt::Let(Def {
            name: name.clone(),
            expr,
        }));
        outputs.push(port(&name, ty, None, false));
        if ty == Ty::F32 {
            scope.push(name);
        }
    }
    let next = state
        .iter()
        .map(|s| Def {
            name: s.name.clone(),
            expr: num_expr(r, &scope, 3),
        })
        .collect();
    Case {
        c: Compute {
            inputs,
            params: vec![],
            state,
            defs,
            outputs,
            next,
        },
        ins,
    }
}

fn sample(r: &mut Rng, (rg, wild): (Option<[f64; 2]>, bool)) -> f32 {
    match rg {
        Some(_) if wild && r.below(4) == 0 => r.pick(&WILD[..3]),
        Some([lo, hi]) => match r.below(4) {
            0 => lo as f32,
            1 => hi as f32,
            // A value inside the f32 hull of [lo, hi]: what the range means (SEMANTICS §6).
            _ => ((lo + r.unit() * (hi - lo)) as f32).clamp(lo as f32, hi as f32),
        },
        None if r.below(2) == 0 => r.pick(&WILD),
        None => (r.pick(&LITS) * r.unit()) as f32,
    }
}

fn contains(iv: &Val<Iv, Bv>, x: &Val<f32, bool>) -> bool {
    match (iv, x) {
        (Val::N(iv), Val::N(x)) if x.is_nan() => iv.nan,
        (Val::N(iv), Val::N(x)) => iv.lo <= *x as f64 && *x as f64 <= iv.hi,
        (Val::B(bv), Val::B(x)) => {
            if *x {
                bv.t
            } else {
                bv.f
            }
        }
        _ => false,
    }
}

#[test]
fn interval_proof_contains_every_f32_run() {
    let mut r = Rng(0x5eed_1234_abcd_0001);
    for case in 0..3000 {
        let Case { c, ins } = random_case(&mut r);
        let (outs, next) = step_intervals(&c);
        for _ in 0..200 {
            let vals: Vec<f32> = ins.iter().map(|&s| sample(&mut r, s)).collect();
            let (inputs, state) = vals.split_at(c.inputs.len());
            let wrap = |v: &[f32]| v.iter().map(|&x| Val::N(x)).collect();
            let (co, cn) = c.step(&mut F32, wrap(inputs), wrap(state));
            for (what, ivs, xs) in [("output", &outs, &co), ("next", &next, &cn)] {
                for (k, (iv, x)) in ivs.iter().zip(xs).enumerate() {
                    assert!(
                        contains(iv, x),
                        "case {case}: {what} {k} = {x:?} not in {iv:?}\ninputs {inputs:?} state {state:?}\n{c:#?}"
                    );
                }
            }
        }
    }
}
