//! Class 3 (first check): output ranges proved by interval arithmetic over every input the contract allows and
//! every state value, NaN and infinities included. Sound for the f32 firmware: each bound is rounded outward to f32,
//! so it contains what the chip computes, not what exact arithmetic would.

use crate::expr::{Cmp, Compute, Domain, Env, Expr, Op, Ty, V, Val};
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

pub struct Intervals;

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
                (Expr::Var(n), true) => (
                    n,
                    Box::new(|a: Iv| Iv {
                        lo: a.lo.max(-(f32::MAX as f64)),
                        hi: a.hi.min(f32::MAX as f64),
                        nan: false,
                    }),
                ),
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

/// The interval every input may take under its contract, and every state value may take at all.
pub fn step_intervals(c: &Compute) -> (Vec<V<Intervals>>, Vec<V<Intervals>>) {
    let inputs = c
        .inputs
        .iter()
        .map(|p| match (p.ty, p.range) {
            (Ty::Bool, _) => Val::B(Bv { t: true, f: true }),
            (_, Some([lo, hi])) => Val::N(Iv {
                lo: down(lo),
                hi: up(hi),
                nan: false,
            }),
            (Ty::U8, None) => Val::N(Iv {
                lo: 0.0,
                hi: 255.0,
                nan: false,
            }),
            (Ty::F32, None) => Val::N(ANY),
        })
        .collect();
    let state = c
        .init(&mut Intervals)
        .into_iter()
        .map(|v| match v {
            Val::N(_) => Val::N(ANY),
            Val::B(_) => Val::B(Bv { t: true, f: true }),
        })
        .collect();
    c.step(&mut Intervals, inputs, state)
}

/// `C3-RANGE`: every output with a declared range stays inside it and is never NaN, for every allowed input and
/// every state (so it holds on every firing, whatever happened before).
pub fn check(ir: &Ir) -> Vec<Violation> {
    let mut bad = Vec::new();
    for blk in &ir.blocks {
        let Some(c) = &blk.compute else { continue };
        let (outs, _) = step_intervals(c);
        for (p, o) in c.outputs.iter().zip(outs) {
            let (Some([lo, hi]), Val::N(iv)) = (p.range, o) else {
                continue;
            };
            if iv.nan || (!iv.empty() && (iv.lo < lo || iv.hi > hi)) {
                bad.push(v(
                    "range",
                    "C3-RANGE",
                    format!(
                        "{}.{}: can be [{}, {}]{} outside declared [{lo}, {hi}]",
                        blk.id,
                        p.name,
                        iv.lo,
                        iv.hi,
                        if iv.nan { " or NaN" } else { "" }
                    ),
                ));
            }
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
        Intervals.op(o, a, b)
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
                })
                .into(),
            state: vec![],
            defs: vec![
                Def {
                    name: "lim".into(),
                    expr: lim,
                },
                Def {
                    name: "setpoint".into(),
                    expr: if guarded {
                        select(var("wanted").is_finite(), clamp, num(0.0))
                    } else {
                        clamp
                    },
                },
            ],
            outputs: vec![Port {
                name: "setpoint".into(),
                ty: Ty::F32,
                unit: Some("A".into()),
                range: Some([-8.0, 8.0]),
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
