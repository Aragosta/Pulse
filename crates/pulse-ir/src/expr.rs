//! Block behaviour as data: a small, total expression language over numbers and booleans. One tree walk
//! (`Compute::step`) is shared by every interpretation, each a `Domain`: f32 is what the firmware computes, f64 is the
//! model, intervals prove bounds (`class3`), Rust source is the firmware (`rust`). They cannot drift apart because they
//! are the same walk. No loops, no calls, no rewriting: `0 * x` stays `0 * x`, since it is NaN when `x` is.

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Expr {
    Num(f64),
    Bool(bool),
    Var(String),
    Neg(Box<Expr>),
    Add(Box<Expr>, Box<Expr>),
    Sub(Box<Expr>, Box<Expr>),
    Mul(Box<Expr>, Box<Expr>),
    Div(Box<Expr>, Box<Expr>),
    /// Rust `f32::max`/`min`: a NaN operand is ignored and the other one returned.
    Max(Box<Expr>, Box<Expr>),
    Min(Box<Expr>, Box<Expr>),
    Lt(Box<Expr>, Box<Expr>),
    Le(Box<Expr>, Box<Expr>),
    Gt(Box<Expr>, Box<Expr>),
    Ge(Box<Expr>, Box<Expr>),
    Eq(Box<Expr>, Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Or(Box<Expr>, Box<Expr>),
    Not(Box<Expr>),
    IsFinite(Box<Expr>),
    /// `if c { a } else { b }`. Both sides are pure, so which one is evaluated never matters.
    Select(Box<Expr>, Box<Expr>, Box<Expr>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Ty {
    F32,
    Bool,
    /// Input only (e.g. a state code); read as a number.
    U8,
}

/// An input or output of a block. On an input `range` is an assumption (finite, within bounds); on an output it is
/// an obligation `class3` must prove. `None`: anything, including NaN and infinities.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Port {
    pub name: String,
    pub ty: Ty,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub range: Option<[f64; 2]>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Def {
    pub name: String,
    pub expr: Expr,
}

/// What a block computes on each firing: `defs` in order (each may use inputs, state and earlier defs); outputs
/// name a def, input or state; `next` gives new state values (a state not listed keeps its value).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Compute {
    pub inputs: Vec<Port>,
    /// Initial values: `Num` or `Bool` literals.
    pub state: Vec<Def>,
    pub defs: Vec<Def>,
    pub outputs: Vec<Port>,
    pub next: Vec<Def>,
}

// ---- building expressions (what a frontend writes) ------------------------------------------------------------------

pub fn var(name: &str) -> Expr {
    Expr::Var(name.into())
}
pub fn num(x: f64) -> Expr {
    Expr::Num(x)
}
pub fn select(c: Expr, a: Expr, b: Expr) -> Expr {
    Expr::Select(c.into(), a.into(), b.into())
}
macro_rules! ops {
    ($($trait:ident $method:ident $variant:ident),*) => {$(
        impl std::ops::$trait for Expr {
            type Output = Expr;
            fn $method(self, rhs: Expr) -> Expr {
                Expr::$variant(self.into(), rhs.into())
            }
        }
    )*};
}
ops!(Add add Add, Sub sub Sub, Mul mul Mul, Div div Div);
impl std::ops::Neg for Expr {
    type Output = Expr;
    fn neg(self) -> Expr {
        Expr::Neg(self.into())
    }
}
macro_rules! methods {
    ($($method:ident $variant:ident),*) => {
        impl Expr {$(
            pub fn $method(self, rhs: Expr) -> Expr {
                Expr::$variant(self.into(), rhs.into())
            }
        )*}
    };
}
methods!(max Max, min Min, lt Lt, le Le, gt Gt, ge Ge, eq Eq, and And, or Or);
impl Expr {
    #[allow(clippy::should_implement_trait)]
    pub fn not(self) -> Expr {
        Expr::Not(self.into())
    }
    pub fn is_finite(self) -> Expr {
        Expr::IsFinite(self.into())
    }
}

// ---- one walk, many interpretations ----------------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
pub enum Val<N, B> {
    N(N),
    B(B),
}
pub type V<D> = Val<<D as Domain>::N, <D as Domain>::B>;
pub type Env<D> = Vec<(String, V<D>)>;

#[derive(Clone, Copy, Debug)]
pub enum Op {
    Add,
    Sub,
    Mul,
    Div,
    Max,
    Min,
}
#[derive(Clone, Copy, Debug)]
pub enum Cmp {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
}

pub trait Domain: Sized {
    type N: Clone;
    type B: Clone;
    fn num(&mut self, x: f64) -> Self::N;
    fn boolean(&mut self, b: bool) -> Self::B;
    fn neg(&mut self, a: Self::N) -> Self::N;
    fn op(&mut self, op: Op, a: Self::N, b: Self::N) -> Self::N;
    fn cmp(&mut self, cmp: Cmp, a: Self::N, b: Self::N) -> Self::B;
    /// `and == true`: and, else or.
    fn logic(&mut self, and: bool, a: Self::B, b: Self::B) -> Self::B;
    fn not(&mut self, a: Self::B) -> Self::B;
    fn is_finite(&mut self, a: Self::N) -> Self::B;
    fn select(&mut self, c: Self::B, a: V<Self>, b: V<Self>) -> V<Self>;
    /// Called on each def and next-state value; codegen turns it into a `let`.
    fn bind(&mut self, _next: bool, _name: &str, v: V<Self>) -> V<Self> {
        v
    }
    /// The environment to evaluate one side of a `Select` in, knowing `cond == holds`. `None`: unchanged.
    fn assume(&mut self, _cond: &Expr, _holds: bool, _env: &Env<Self>) -> Option<Env<Self>> {
        None
    }
}

fn lookup<'a, T>(env: &'a [(String, T)], name: &str) -> &'a T {
    &env.iter()
        .rev()
        .find(|(n, _)| n == name)
        .expect("validated")
        .1
}
fn n<D: Domain>(v: V<D>) -> D::N {
    match v {
        Val::N(x) => x,
        Val::B(_) => unreachable!("validated"),
    }
}
fn b<D: Domain>(v: V<D>) -> D::B {
    match v {
        Val::B(x) => x,
        Val::N(_) => unreachable!("validated"),
    }
}

impl Expr {
    pub fn eval<D: Domain>(&self, d: &mut D, env: &Env<D>) -> V<D> {
        use Expr::*;
        let num = |e: &Expr, d: &mut D| n::<D>(e.eval(d, env));
        match self {
            Num(x) => Val::N(d.num(*x)),
            Bool(x) => Val::B(d.boolean(*x)),
            Var(name) => lookup(env, name).clone(),
            Neg(a) => {
                let a = num(a, d);
                Val::N(d.neg(a))
            }
            Add(x, y) | Sub(x, y) | Mul(x, y) | Div(x, y) | Max(x, y) | Min(x, y) => {
                let op = match self {
                    Add(..) => Op::Add,
                    Sub(..) => Op::Sub,
                    Mul(..) => Op::Mul,
                    Div(..) => Op::Div,
                    Max(..) => Op::Max,
                    _ => Op::Min,
                };
                let (x, y) = (num(x, d), num(y, d));
                Val::N(d.op(op, x, y))
            }
            Lt(x, y) | Le(x, y) | Gt(x, y) | Ge(x, y) | Eq(x, y) => {
                let cmp = match self {
                    Lt(..) => Cmp::Lt,
                    Le(..) => Cmp::Le,
                    Gt(..) => Cmp::Gt,
                    Ge(..) => Cmp::Ge,
                    _ => Cmp::Eq,
                };
                let (x, y) = (num(x, d), num(y, d));
                Val::B(d.cmp(cmp, x, y))
            }
            And(x, y) | Or(x, y) => {
                let (x, y) = (b::<D>(x.eval(d, env)), b::<D>(y.eval(d, env)));
                Val::B(d.logic(matches!(self, And(..)), x, y))
            }
            Not(a) => {
                let a = b::<D>(a.eval(d, env));
                Val::B(d.not(a))
            }
            IsFinite(a) => {
                let a = num(a, d);
                Val::B(d.is_finite(a))
            }
            Select(c, x, y) => {
                let cv = b::<D>(c.eval(d, env));
                let ex = d.assume(c, true, env);
                let ey = d.assume(c, false, env);
                let x = x.eval(d, ex.as_ref().unwrap_or(env));
                let y = y.eval(d, ey.as_ref().unwrap_or(env));
                d.select(cv, x, y)
            }
        }
    }

    /// Type of the expression given the types in scope (`U8` reads as `F32`).
    fn ty(&self, scope: &[(&str, Ty)]) -> Result<Ty, String> {
        use Expr::*;
        let want = |e: &Expr, t: Ty| -> Result<(), String> {
            let got = e.ty(scope)?;
            if got == t {
                Ok(())
            } else {
                Err(format!("expected {t:?}, got {got:?} in {e:?}"))
            }
        };
        Ok(match self {
            Num(_) => Ty::F32,
            Bool(_) => Ty::Bool,
            Var(name) => match scope.iter().rev().find(|(n, _)| n == name) {
                Some((_, Ty::U8)) => Ty::F32,
                Some((_, t)) => *t,
                None => return Err(format!("unknown name {name:?}")),
            },
            Neg(a) => {
                want(a, Ty::F32)?;
                Ty::F32
            }
            Add(x, y) | Sub(x, y) | Mul(x, y) | Div(x, y) | Max(x, y) | Min(x, y) => {
                want(x, Ty::F32)?;
                want(y, Ty::F32)?;
                Ty::F32
            }
            Lt(x, y) | Le(x, y) | Gt(x, y) | Ge(x, y) | Eq(x, y) => {
                want(x, Ty::F32)?;
                want(y, Ty::F32)?;
                Ty::Bool
            }
            And(x, y) | Or(x, y) => {
                want(x, Ty::Bool)?;
                want(y, Ty::Bool)?;
                Ty::Bool
            }
            Not(a) => {
                want(a, Ty::Bool)?;
                Ty::Bool
            }
            IsFinite(a) => {
                want(a, Ty::F32)?;
                Ty::Bool
            }
            Select(c, x, y) => {
                want(c, Ty::Bool)?;
                let t = x.ty(scope)?;
                want(y, t)?;
                t
            }
        })
    }
}

impl Compute {
    /// One firing: `(outputs, next state)`, in the order of `self.outputs` and `self.state`.
    pub fn step<D: Domain>(
        &self,
        d: &mut D,
        inputs: Vec<V<D>>,
        state: Vec<V<D>>,
    ) -> (Vec<V<D>>, Vec<V<D>>) {
        let mut env: Env<D> = self
            .inputs
            .iter()
            .map(|p| p.name.clone())
            .zip(inputs)
            .collect();
        env.extend(self.state.iter().map(|s| s.name.clone()).zip(state));
        for def in &self.defs {
            let v = def.expr.eval(d, &env);
            let v = d.bind(false, &def.name, v);
            env.push((def.name.clone(), v));
        }
        let outputs = self
            .outputs
            .iter()
            .map(|p| lookup(&env, &p.name).clone())
            .collect();
        let next = self
            .state
            .iter()
            .map(|s| match self.next.iter().find(|x| x.name == s.name) {
                Some(x) => {
                    let v = x.expr.eval(d, &env);
                    d.bind(true, &s.name, v)
                }
                None => lookup(&env, &s.name).clone(),
            })
            .collect();
        (outputs, next)
    }

    /// Initial state, in the order of `self.state`.
    pub fn init<D: Domain>(&self, d: &mut D) -> Vec<V<D>> {
        self.state
            .iter()
            .map(|s| s.expr.eval(d, &Vec::new()))
            .collect()
    }

    /// Well-formedness: identifier names, unique across inputs/state/defs; every expression types; outputs and
    /// next-state values have their declared types. Errors are human-readable; `Ir::validate` wraps them.
    pub fn validate(&self) -> Vec<String> {
        let mut bad = Vec::new();
        let mut scope: Vec<(&str, Ty)> = Vec::new();
        let declare = |scope: &mut Vec<(&str, Ty)>, name: &'_ str, ty, bad: &mut Vec<String>| {
            if !crate::is_ident(name) {
                bad.push(format!("{name:?} is not an identifier"));
            }
            if scope.iter().any(|(n, _)| *n == name) {
                bad.push(format!("{name:?} declared twice"));
            }
            ty
        };
        for p in &self.inputs {
            let t = declare(&mut scope, &p.name, p.ty, &mut bad);
            scope.push((&p.name, t));
            if p.range
                .is_some_and(|[lo, hi]| lo.is_nan() || hi.is_nan() || lo > hi)
            {
                bad.push(format!("input {}: bad range {:?}", p.name, p.range));
            }
        }
        for s in &self.state {
            let t = match s.expr {
                Expr::Num(_) => Ty::F32,
                Expr::Bool(_) => Ty::Bool,
                _ => {
                    bad.push(format!("state {}: initial value must be a literal", s.name));
                    Ty::F32
                }
            };
            let t = declare(&mut scope, &s.name, t, &mut bad);
            scope.push((&s.name, t));
        }
        for def in &self.defs {
            let t = def.expr.ty(&scope).unwrap_or_else(|e| {
                bad.push(format!("{}: {e}", def.name));
                Ty::F32
            });
            let t = declare(&mut scope, &def.name, t, &mut bad);
            scope.push((&def.name, t));
        }
        for p in &self.outputs {
            match scope.iter().rev().find(|(n, _)| *n == p.name) {
                None => bad.push(format!("output {}: no such def, input or state", p.name)),
                Some((_, t)) if *t != p.ty || p.ty == Ty::U8 => bad.push(format!(
                    "output {}: declared {:?}, computes {t:?}",
                    p.name, p.ty
                )),
                _ => {}
            }
        }
        for (i, x) in self.next.iter().enumerate() {
            let Some(s) = self.state.iter().position(|s| s.name == x.name) else {
                bad.push(format!("next {}: not a state variable", x.name));
                continue;
            };
            if self.next[..i].iter().any(|o| o.name == x.name) {
                bad.push(format!("next {}: assigned twice", x.name));
            }
            let want = scope
                .iter()
                .find(|(n, _)| *n == self.state[s].name)
                .map(|(_, t)| *t);
            match x.expr.ty(&scope) {
                Ok(t) if Some(t) == want => {}
                Ok(t) => bad.push(format!(
                    "next {}: computes {t:?}, state is {want:?}",
                    x.name
                )),
                Err(e) => bad.push(format!("next {}: {e}", x.name)),
            }
        }
        bad
    }
}

// ---- concrete arithmetic: f32 (the firmware) and f64 (the model) ----------------------------------------------------

macro_rules! float_domain {
    ($name:ident, $t:ty) => {
        pub struct $name;
        impl Domain for $name {
            type N = $t;
            type B = bool;
            fn num(&mut self, x: f64) -> $t {
                x as $t
            }
            fn boolean(&mut self, b: bool) -> bool {
                b
            }
            fn neg(&mut self, a: $t) -> $t {
                -a
            }
            fn op(&mut self, op: Op, a: $t, b: $t) -> $t {
                match op {
                    Op::Add => a + b,
                    Op::Sub => a - b,
                    Op::Mul => a * b,
                    Op::Div => a / b,
                    Op::Max => a.max(b),
                    Op::Min => a.min(b),
                }
            }
            fn cmp(&mut self, cmp: Cmp, a: $t, b: $t) -> bool {
                match cmp {
                    Cmp::Lt => a < b,
                    Cmp::Le => a <= b,
                    Cmp::Gt => a > b,
                    Cmp::Ge => a >= b,
                    Cmp::Eq => a == b,
                }
            }
            fn logic(&mut self, and: bool, a: bool, b: bool) -> bool {
                if and { a && b } else { a || b }
            }
            fn not(&mut self, a: bool) -> bool {
                !a
            }
            fn is_finite(&mut self, a: $t) -> bool {
                a.is_finite()
            }
            fn select(&mut self, c: bool, a: V<Self>, b: V<Self>) -> V<Self> {
                if c { a } else { b }
            }
        }
    };
}
float_domain!(F32, f32);
float_domain!(F64, f64);
