//! Class 3 (first checks): state invariants and output ranges, proved by interval arithmetic over every input the
//! contract allows, NaN and infinities included. Sound for the f32 firmware: each bound is rounded outward to f32,
//! so it contains what the chip computes, not what exact arithmetic would.

use crate::expr::{Cmp, Compute, Def, Domain, Env, Expr, Op, Port, Ty, V, Val};
use crate::{Ir, Violation, v};

/// A set of f32 values: `[lo, hi]` (empty when `lo > hi`, infinities allowed), plus NaN if `nan`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Iv {
    pub lo: f64,
    pub hi: f64,
    pub nan: bool,
}
/// Which truth values a condition can take.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bv {
    pub t: bool,
    pub f: bool,
}

const ANY: Iv = Iv {
    lo: f64::NEG_INFINITY,
    hi: f64::INFINITY,
    nan: true,
};
const EMPTY: Iv = Iv {
    lo: f64::INFINITY,
    hi: f64::NEG_INFINITY,
    nan: false,
};

/// Largest f32 <= x, and smallest f32 >= x.
fn down(x: f64) -> f64 {
    let f = x as f32;
    (if f as f64 > x { f.next_down() } else { f }) as f64
}
fn up(x: f64) -> f64 {
    let f = x as f32;
    (if (f as f64) < x { f.next_up() } else { f }) as f64
}

impl Iv {
    pub fn point(x: f64) -> Iv {
        let x = x as f32 as f64;
        Iv {
            lo: x,
            hi: x,
            nan: false,
        }
    }
    fn empty(&self) -> bool {
        self.lo > self.hi
    }
    fn has(&self, x: f64) -> bool {
        self.lo <= x && x <= self.hi
    }
    fn has_inf(&self) -> bool {
        !self.empty() && (self.lo == f64::NEG_INFINITY || self.hi == f64::INFINITY)
    }
    fn hull(self, o: Iv) -> Iv {
        Iv {
            lo: self.lo.min(o.lo),
            hi: self.hi.max(o.hi),
            nan: self.nan || o.nan,
        }
    }
}

/// `finite`: for inputs that may glitch, the range of their finite samples, so `is_finite(x)` narrows `x` back to it.
#[derive(Default)]
pub struct Intervals {
    finite: Vec<(String, Iv)>,
}

impl Domain for Intervals {
    type N = Iv;
    type B = Bv;
    fn num(&mut self, x: f64) -> Iv {
        Iv::point(x)
    }
    fn boolean(&mut self, b: bool) -> Bv {
        Bv { t: b, f: !b }
    }
    fn neg(&mut self, a: Iv) -> Iv {
        Iv {
            lo: -a.hi,
            hi: -a.lo,
            nan: a.nan,
        }
    }
    fn op(&mut self, op: Op, a: Iv, b: Iv) -> Iv {
        if let Op::Max | Op::Min = op {
            // Rust semantics: NaN on one side returns the other side; NaN only if both are.
            let f = |x: f64, y: f64| if let Op::Max = op { x.max(y) } else { x.min(y) };
            let mut r = if a.empty() || b.empty() {
                EMPTY
            } else {
                Iv {
                    lo: f(a.lo, b.lo),
                    hi: f(a.hi, b.hi),
                    nan: false,
                }
            };
            if a.nan {
                r = r.hull(Iv { nan: false, ..b });
            }
            if b.nan {
                r = r.hull(Iv { nan: false, ..a });
            }
            r.nan = a.nan && b.nan;
            return r;
        }
        let mut nan = a.nan || b.nan;
        if a.empty() || b.empty() {
            return Iv { nan, ..EMPTY };
        }
        let exact = |x: f64, y: f64| match op {
            Op::Add => x + y,
            Op::Sub => x - y,
            Op::Mul => x * y,
            _ => x / y,
        };
        let (lo, hi) = match op {
            // Division by a set containing zero: anything, NaN if 0/0 is possible.
            Op::Div if b.has(0.0) => {
                nan |= a.has(0.0) || (a.has_inf() && b.has_inf());
                (f64::NEG_INFINITY, f64::INFINITY)
            }
            _ => {
                // Away from NaN points each op is monotone in each argument, so the corners bound it.
                // Dropped NaN corners (inf - inf, 0 * inf, inf / inf) are recorded in `nan`.
                match op {
                    Op::Mul => nan |= (a.has(0.0) && b.has_inf()) || (b.has(0.0) && a.has_inf()),
                    Op::Div => nan |= a.has_inf() && b.has_inf(),
                    _ => {}
                }
                let mut r = EMPTY;
                for x in [a.lo, a.hi] {
                    for y in [b.lo, b.hi] {
                        let z = exact(x, y);
                        if z.is_nan() {
                            nan = true;
                        } else {
                            r = r.hull(Iv {
                                lo: z,
                                hi: z,
                                nan: false,
                            });
                        }
                    }
                }
                // f32 * f32 is exact in f64; +, -, / may round, so widen by one f64 step first.
                match op {
                    // A dropped 0 * inf or inf / inf corner hides interior values (0 * finite = 0, finite / inf = 0).
                    // ponytail: gives up to the whole line then; exact case analysis if a proof ever needs it.
                    Op::Mul if (a.has(0.0) && b.has_inf()) || (b.has(0.0) && a.has_inf()) => {
                        (f64::NEG_INFINITY, f64::INFINITY)
                    }
                    Op::Div if a.has_inf() && b.has_inf() => (f64::NEG_INFINITY, f64::INFINITY),
                    Op::Mul => (r.lo, r.hi),
                    _ => (r.lo.next_down(), r.hi.next_up()),
                }
            }
        };
        Iv {
            lo: down(lo),
            hi: up(hi),
            nan,
        }
    }
    fn cmp(&mut self, cmp: Cmp, a: Iv, b: Iv) -> Bv {
        let some = !a.empty() && !b.empty();
        let nan = a.nan || b.nan;
        let (t, f) = match cmp {
            Cmp::Lt => (a.lo < b.hi, a.hi >= b.lo),
            Cmp::Le => (a.lo <= b.hi, a.hi > b.lo),
            Cmp::Gt => (a.hi > b.lo, a.lo <= b.hi),
            Cmp::Ge => (a.hi >= b.lo, a.lo < b.hi),
            Cmp::Eq => (
                a.lo <= b.hi && b.lo <= a.hi,
                !(a.lo == a.hi && b.lo == b.hi && a.lo == b.lo),
            ),
        };
        Bv {
            t: some && t,
            f: (some && f) || nan,
        }
    }
    fn logic(&mut self, and: bool, a: Bv, b: Bv) -> Bv {
        if and {
            Bv {
                t: a.t && b.t,
                f: a.f || b.f,
            }
        } else {
            Bv {
                t: a.t || b.t,
                f: a.f && b.f,
            }
        }
    }
    fn not(&mut self, a: Bv) -> Bv {
        Bv { t: a.f, f: a.t }
    }
    fn is_finite(&mut self, a: Iv) -> Bv {
        // Two distinct f32 endpoints always enclose a finite value; a single point is finite unless it is +-inf.
        let finite = !a.empty() && (a.lo < a.hi || a.lo.is_finite());
        Bv {
            t: finite,
            f: a.nan || a.has_inf(),
        }
    }
    fn select(&mut self, c: Bv, a: V<Self>, b: V<Self>) -> V<Self> {
        match (a, b) {
            (Val::N(x), Val::N(y)) => Val::N(match (c.t, c.f) {
                (true, true) => x.hull(y),
                (true, false) => x,
                (false, true) => y,
                _ => EMPTY,
            }),
            (Val::B(x), Val::B(y)) => Val::B(Bv {
                t: (c.t && x.t) || (c.f && y.t),
                f: (c.t && x.f) || (c.f && y.f),
            }),
            _ => unreachable!("validated"),
        }
    }
    /// Inside one side of a `Select`, narrow a variable compared against a literal (`x >= 0`, `is_finite(x)`), so
    /// the proof keeps what the branch knows. Sound: that side's value is only used when the condition has that value.
    fn assume(&mut self, cond: &Expr, holds: bool, env: &Env<Self>) -> Option<Env<Self>> {
        let (name, refine): (&str, Box<dyn Fn(Iv) -> Iv>) = match cond {
            Expr::IsFinite(x) => match (&**x, holds) {
                (Expr::Var(n), true) => {
                    let fin = self.finite.iter().find(|(f, _)| f == n).map_or(
                        Iv {
                            lo: -(f32::MAX as f64),
                            hi: f32::MAX as f64,
                            nan: false,
                        },
                        |(_, iv)| *iv,
                    );
                    (
                        n,
                        Box::new(move |a: Iv| Iv {
                            lo: a.lo.max(fin.lo),
                            hi: a.hi.min(fin.hi),
                            nan: false,
                        }),
                    )
                }
                _ => return None,
            },
            Expr::Lt(x, y) | Expr::Le(x, y) | Expr::Gt(x, y) | Expr::Ge(x, y) => {
                let (Expr::Var(n), Expr::Num(c)) = (&**x, &**y) else {
                    return None;
                };
                let c = *c as f32 as f64;
                // Does the condition say "x is above c" (Gt/Ge holding, Lt/Le failing)?
                let above = matches!(cond, Expr::Gt(..) | Expr::Ge(..)) == holds;
                // Holding comparisons exclude NaN; failing ones do not (NaN fails every comparison).
                (
                    n,
                    Box::new(move |a: Iv| {
                        let nan = a.nan && !holds;
                        if above {
                            Iv {
                                lo: a.lo.max(c),
                                nan,
                                ..a
                            }
                        } else {
                            Iv {
                                hi: a.hi.min(c),
                                nan,
                                ..a
                            }
                        }
                    }),
                )
            }
            _ => return None,
        };
        let mut env = env.clone();
        let (_, val) = env.iter_mut().rev().find(|(n, _)| n == name)?;
        if let Val::N(a) = *val {
            *val = Val::N(refine(a));
        }
        Some(env)
    }
}

/// One firing over every input the contracts allow and every state inside its invariant range (unranged state:
/// anything, NaN included).
pub fn step_intervals(c: &Compute) -> (Vec<V<Intervals>>, Vec<V<Intervals>>) {
    let mut d = Intervals::default();
    let range = |[lo, hi]: [f64; 2]| Iv {
        lo: down(lo),
        hi: up(hi),
        nan: false,
    };
    let inputs = c
        .inputs
        .iter()
        .map(|p| match (p.ty, p.range) {
            (Ty::Bool, _) => Val::B(Bv { t: true, f: true }),
            (_, Some(r)) if p.glitch => {
                d.finite.push((p.name.clone(), range(r)));
                Val::N(ANY)
            }
            (_, Some(r)) => Val::N(range(r)),
            (Ty::U8, None) => Val::N(range([0.0, 255.0])),
            (Ty::F32, None) => Val::N(ANY),
        })
        .collect();
    let state = c
        .init(&mut d)
        .into_iter()
        .zip(&c.state)
        .map(|(v, s)| match (v, s.range) {
            (Val::B(_), _) => Val::B(Bv { t: true, f: true }),
            (Val::N(_), Some(r)) => Val::N(range(r)),
            (Val::N(_), None) => Val::N(ANY),
        })
        .collect();
    c.step(&mut d, inputs, state)
}

/// Whether `iv` leaves `[lo, hi]`. Values are f32, so a range means its f32 hull (the same hull assumed on entry);
/// otherwise a held, unchanged state would fail an invariant whose bounds are not f32 numbers.
fn outside(iv: Iv, [lo, hi]: [f64; 2]) -> Option<String> {
    (iv.nan || (!iv.empty() && (iv.lo < down(lo) || iv.hi > up(hi)))).then(|| {
        format!(
            "can be [{}, {}]{} outside [{lo}, {hi}]",
            iv.lo,
            iv.hi,
            if iv.nan { " or NaN" } else { "" }
        )
    })
}

/// Invariants and output ranges of one flat compute, for every input its contracts allow. `from` is how many
/// outputs are its own (the rest are probes of inner block outputs); `at` prefixes every message.
fn prove(c: &Compute, own: usize, at: &str) -> Vec<Violation> {
    let mut bad = Vec::new();
    let (outs, next) = step_intervals(c);
    let init = c.init(&mut Intervals::default());
    for ((s, n), i) in c.state.iter().zip(next).zip(init) {
        let (Some(r), Val::N(n), Val::N(i)) = (s.range, n, i) else {
            continue;
        };
        for (when, iv) in [("initially", i), ("after a firing", n)] {
            if let Some(why) = outside(iv, r) {
                let msg = format!("{at}{} {when} {why}", s.name);
                bad.push(v("invariant", "C3-INVARIANT", msg));
            }
        }
    }
    for (k, (p, o)) in c.outputs.iter().zip(outs).enumerate() {
        let (Some(r), Val::N(iv)) = (p.range, o) else {
            continue;
        };
        if let Some(why) = outside(iv, r) {
            let what = if k < own { "output" } else { "block output" };
            bad.push(v(
                "range",
                "C3-RANGE",
                format!("{at}{what} {} {why}", p.name),
            ));
        }
    }
    bad
}

/// Whether a bound value meets a component input's contract. Without `glitch`: never NaN, inside the range. With
/// it: the good (finite) samples inside the range; a value whose finite part is unknown to intervals (a glitchy
/// input) meets it when it is that input and its own contract is inside.
fn breaks(iv: Iv, value: &Expr, contract: &Port, glitchy: &[(String, Iv)]) -> Option<String> {
    let [lo, hi] = contract.range?;
    let inside = |a: Iv| a.empty() || (a.lo >= down(lo) && a.hi <= up(hi));
    let ok = if !contract.glitch {
        !iv.nan && inside(iv)
    } else {
        inside(Iv { nan: false, ..iv })
            || matches!(value, Expr::Var(n) if glitchy.iter().any(|(g, fin)| g == n && inside(*fin)))
    };
    (!ok).then(|| {
        format!(
            "can be [{}, {}]{}, outside the contract {}[{lo}, {hi}]",
            iv.lo,
            iv.hi,
            if iv.nan { " or NaN" } else { "" },
            if contract.glitch {
                "for good samples "
            } else {
                ""
            }
        )
    })
}

/// `C3-INVARIANT`: every state range holds initially, and one firing from inside all of them (any allowed input)
/// stays inside, so by induction it holds on every firing. `C3-RANGE`: under those invariants every declared output
/// range holds and is never NaN. `C3-CONTRACT`: every component instance is fed what its component assumes.
///
/// Each component is proved once, on its own, under its input contracts; the whole firmware (`graph::firmware`) is
/// then proved with inputs between blocks carrying what their producers are proved to output, and every instance's
/// bound inputs are checked against its component's contracts, so each component's own proof applies to it.
pub fn check(ir: &Ir) -> Vec<Violation> {
    let mut bad = Vec::new();
    for comp in &ir.components {
        if let Ok(c) = ir.flat(&comp.compute) {
            bad.extend(prove(
                &c,
                c.outputs.len(),
                &format!("component {}: ", comp.name),
            ));
        }
    }
    let fw = match crate::graph::firmware(ir) {
        Ok(fw) => fw,
        Err(es) => {
            bad.extend(es.into_iter().map(|e| v("graph", "IR-GRAPH", e)));
            return bad;
        }
    };
    let mut c = fw.compute;
    let own = c.outputs.len();
    c.outputs.extend(fw.probes);
    // Each obligation's bound value becomes a probe, so the same interval pass yields it.
    let first = c.outputs.len();
    for (i, o) in fw.obligations.iter().enumerate() {
        let name = format!("__obligation_{i}");
        c.defs.push(
            Def {
                name: name.clone(),
                expr: o.value.clone(),
            }
            .into(),
        );
        c.outputs.push(Port {
            name,
            range: None,
            glitch: false,
            ..o.contract.clone()
        });
    }
    bad.extend(prove(&c, own, ""));
    let glitchy: Vec<(String, Iv)> = c
        .inputs
        .iter()
        .filter(|p| p.glitch)
        .filter_map(|p| {
            Some((
                p.name.clone(),
                Iv {
                    lo: down(p.range?[0]),
                    hi: up(p.range?[1]),
                    nan: false,
                },
            ))
        })
        .collect();
    let (outs, _) = step_intervals(&c);
    for (o, val) in fw.obligations.iter().zip(&outs[first..]) {
        if let Val::N(iv) = val
            && let Some(why) = breaks(*iv, &o.value, &o.contract, &glitchy)
        {
            let msg = format!("{} ({}): {why}", o.at, o.component);
            bad.push(v("contract", "C3-CONTRACT", msg));
        }
    }
    bad
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::expr::*;

    fn iv(lo: f64, hi: f64) -> Iv {
        Iv { lo, hi, nan: false }
    }
    fn op(o: Op, a: Iv, b: Iv) -> Iv {
        Intervals::default().op(o, a, b)
    }

    /// Every interval result must contain the f32 result for sampled points of its inputs.
    #[test]
    fn interval_ops_contain_every_f32_result() {
        let pts = [
            f32::NEG_INFINITY,
            -1e30,
            -3.5,
            -1.0,
            -1e-30,
            -0.0,
            0.0,
            1e-30,
            0.1,
            1.0,
            3.5,
            1e30,
            f32::MAX,
            f32::INFINITY,
            f32::NAN,
        ];
        let sets: Vec<(Iv, Vec<f32>)> = pts
            .iter()
            .flat_map(|&x| pts.iter().map(move |&y| (x, y)))
            .map(|(x, y)| {
                let (a, b) = if x.is_nan() || y.is_nan() {
                    (x, x)
                } else {
                    (x.min(y), x.max(y))
                };
                let nan = x.is_nan() || y.is_nan();
                let iv = if a.is_nan() {
                    EMPTY
                } else {
                    iv(a as f64, b as f64)
                };
                let members = pts
                    .iter()
                    .copied()
                    .filter(|p| (p.is_nan() && nan) || (*p >= a && *p <= b))
                    .collect();
                (Iv { nan, ..iv }, members)
            })
            .collect();
        for o in [Op::Add, Op::Sub, Op::Mul, Op::Div, Op::Max, Op::Min] {
            for (a, am) in &sets {
                for (b, bm) in &sets {
                    let r = op(o, *a, *b);
                    for &x in am {
                        for &y in bm {
                            let z = F32.op(o, x, y);
                            let ok = if z.is_nan() { r.nan } else { r.has(z as f64) };
                            assert!(ok, "{o:?} {a:?} {b:?}: {x} {y} -> {z} not in {r:?}");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn rounding_is_outward_to_f32() {
        let r = op(
            Op::Add,
            iv(0.1f32 as f64, 0.1f32 as f64),
            iv(0.2f32 as f64, 0.2f32 as f64),
        );
        let f = 0.1f32 + 0.2f32;
        assert!(r.has(f as f64) && r.lo < r.hi);
        assert_eq!(op(Op::Mul, iv(8.0, 8.0), iv(-1.0, 1.0)), iv(-8.0, 8.0)); // exact stays exact
    }

    fn clamp_block(guarded: bool) -> Compute {
        // setpoint = wanted clamped to +-lim, lim = 8 * scale; NaN-guarded or not.
        let lim = if guarded {
            select(
                var("scale").ge(num(0.0)),
                num(8.0) * var("scale").min(num(1.0)),
                num(0.0),
            )
        } else {
            num(8.0) * var("scale")
        };
        let clamp = var("wanted").max(-var("lim")).min(var("lim"));
        Compute {
            inputs: ["wanted", "scale"]
                .map(|n| Port {
                    name: n.into(),
                    ty: Ty::F32,
                    unit: None,
                    range: None,
                    glitch: false,
                })
                .into(),
            params: vec![],
            state: vec![],
            defs: vec![
                Def {
                    name: "lim".into(),
                    expr: lim,
                }
                .into(),
                Def {
                    name: "setpoint".into(),
                    expr: if guarded {
                        select(var("wanted").is_finite(), clamp, num(0.0))
                    } else {
                        clamp
                    },
                }
                .into(),
            ],
            outputs: vec![Port {
                name: "setpoint".into(),
                ty: Ty::F32,
                unit: Some("A".into()),
                range: Some([-8.0, 8.0]),
                glitch: false,
            }],
            next: vec![],
        }
    }

    #[test]
    fn guarded_clamp_proved_unguarded_refuted() {
        let out = |c: &Compute| match step_intervals(c).0.remove(0) {
            Val::N(iv) => iv,
            _ => unreachable!(),
        };
        assert_eq!(out(&clamp_block(true)), iv(-8.0, 8.0));
        let bad = out(&clamp_block(false));
        assert!(bad.nan && bad.hi == f64::INFINITY, "{bad:?}");
    }
}
