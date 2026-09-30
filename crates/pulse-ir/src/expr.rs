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
    /// Input only: besides `range`, a sample may be NaN or +-inf (a bad reading) that the block must guard.
    #[serde(default, skip_serializing_if = "is_false")]
    pub glitch: bool,
}

fn is_false(b: &bool) -> bool {
    !*b
}

/// A state variable: its initial value (a `Num` or `Bool` literal) and, optionally, an invariant range that
/// `class3` proves inductively (it holds initially, and one firing from inside every range stays inside).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StateVar {
    pub name: String,
    pub init: Expr,
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

/// A named constant. Generated code inlines its value, so naming it changes no arithmetic.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Param {
    pub name: String,
    pub value: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
}

/// An instance of a library component (`Ir::components`): its inputs bound to expressions in this scope. After
/// `Compute::flatten` its params, state and defs appear here as `{name}__{x}`; this scope may read only its outputs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Use {
    pub name: String,
    pub component: String,
    pub bind: Vec<Def>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Stmt {
    Let(Def),
    Use(Use),
    /// A state machine, lowered by `Compute::flatten` (`crate::fsm`).
    Fsm(crate::fsm::Fsm),
}
impl From<Def> for Stmt {
    fn from(d: Def) -> Stmt {
        Stmt::Let(d)
    }
}

/// What a block computes on each firing: `defs` in order (each may use inputs, params, state and earlier defs);
/// outputs name a def, input or state; `next` gives new state values (a state not listed keeps its value).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Compute {
    pub inputs: Vec<Port>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub params: Vec<Param>,
    pub state: Vec<StateVar>,
    pub defs: Vec<Stmt>,
    pub outputs: Vec<Port>,
    pub next: Vec<Def>,
}

/// A reusable piece of behaviour (e.g. a PID), instantiated by `Stmt::Use`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Component {
    pub name: String,
    pub compute: Compute,
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

    /// The same expression with every variable replaced by `f(name)`.
    pub fn map_vars(&self, f: &impl Fn(&str) -> Expr) -> Expr {
        use Expr::*;
        let m = |e: &Expr| Box::new(e.map_vars(f));
        match self {
            Num(_) | Bool(_) => self.clone(),
            Var(n) => f(n),
            Neg(a) => Neg(m(a)),
            Not(a) => Not(m(a)),
            IsFinite(a) => IsFinite(m(a)),
            Add(x, y) => Add(m(x), m(y)),
            Sub(x, y) => Sub(m(x), m(y)),
            Mul(x, y) => Mul(m(x), m(y)),
            Div(x, y) => Div(m(x), m(y)),
            Max(x, y) => Max(m(x), m(y)),
            Min(x, y) => Min(m(x), m(y)),
            Lt(x, y) => Lt(m(x), m(y)),
            Le(x, y) => Le(m(x), m(y)),
            Gt(x, y) => Gt(m(x), m(y)),
            Ge(x, y) => Ge(m(x), m(y)),
            Eq(x, y) => Eq(m(x), m(y)),
            And(x, y) => And(m(x), m(y)),
            Or(x, y) => Or(m(x), m(y)),
            Select(c, x, y) => Select(m(c), m(x), m(y)),
        }
    }

    /// Every variable name the expression reads.
    pub fn vars(&self) -> Vec<String> {
        let names = std::cell::RefCell::new(Vec::new());
        self.map_vars(&|n| {
            names.borrow_mut().push(n.to_string());
            var(n)
        });
        names.into_inner()
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
        for p in &self.params {
            let v = Val::N(d.num(p.value));
            env.push((p.name.clone(), v));
        }
        env.extend(self.state.iter().map(|s| s.name.clone()).zip(state));
        for stmt in &self.defs {
            let Stmt::Let(def) = stmt else {
                panic!("flatten before evaluating");
            };
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

    /// Inline every component instance (recursively), prefixing its names with `{instance}__`. Everything that
    /// evaluates, proves or generates code works on the flat form. User names must not contain `__`.
    pub fn flatten(&self, lib: &[Component]) -> Result<Compute, String> {
        self.flatten_at(lib, 0)
    }

    fn flatten_at(&self, lib: &[Component], depth: usize) -> Result<Compute, String> {
        if depth > 16 {
            return Err("components nested more than 16 deep (a component uses itself?)".into());
        }
        let mut flat = Compute {
            defs: Vec::new(),
            ..self.clone()
        };
        for stmt in &self.defs {
            let u = match stmt {
                Stmt::Let(d) => {
                    flat.defs.push(Stmt::Let(d.clone()));
                    continue;
                }
                Stmt::Fsm(f) => {
                    let (params, state, def, update) = f.lower();
                    flat.params.extend(params);
                    flat.state.push(state);
                    flat.defs.push(Stmt::Let(def));
                    flat.next.push(update);
                    continue;
                }
                Stmt::Use(u) => u,
            };
            let comp = lib
                .iter()
                .find(|c| c.name == u.component)
                .ok_or(format!("{}: no component {:?}", u.name, u.component))?;
            let c = comp.compute.flatten_at(lib, depth + 1)?;
            let pre = |n: &str| format!("{}__{n}", u.name);
            // An input bound to a variable or literal is substituted where it is used (no copy, so a contract on
            // that variable still reaches the proof); any other binding becomes a def where the instance stands.
            let mut subst: Vec<(&str, Expr)> = Vec::new();
            for p in &c.inputs {
                let b = u
                    .bind
                    .iter()
                    .find(|b| b.name == p.name)
                    .ok_or(format!("{}: input {} not bound", u.name, p.name))?;
                if let Expr::Var(_) | Expr::Num(_) | Expr::Bool(_) = b.expr {
                    subst.push((&p.name, b.expr.clone()));
                } else {
                    flat.defs.push(Stmt::Let(Def {
                        name: pre(&p.name),
                        expr: b.expr.clone(),
                    }));
                }
            }
            let rename = |e: &Expr| {
                e.map_vars(&|n| match subst.iter().find(|(s, _)| *s == n) {
                    Some((_, e)) => e.clone(),
                    None => var(&pre(n)),
                })
            };
            if let Some(b) = u
                .bind
                .iter()
                .find(|b| !c.inputs.iter().any(|p| p.name == b.name))
            {
                return Err(format!(
                    "{}: {} is not an input of {}",
                    u.name, b.name, u.component
                ));
            }
            flat.params.extend(c.params.iter().map(|p| Param {
                name: pre(&p.name),
                ..p.clone()
            }));
            flat.state.extend(c.state.iter().map(|s| StateVar {
                name: pre(&s.name),
                ..s.clone()
            }));
            for d in &c.defs {
                let Stmt::Let(d) = d else {
                    unreachable!("flattened")
                };
                flat.defs.push(Stmt::Let(Def {
                    name: pre(&d.name),
                    expr: rename(&d.expr),
                }));
            }
            flat.next.extend(c.next.iter().map(|d| Def {
                name: pre(&d.name),
                expr: rename(&d.expr),
            }));
            // An output that passes a substituted input straight through still needs its `{name}__{x}`.
            for o in &c.outputs {
                if let Some((_, e)) = subst.iter().find(|(s, _)| *s == o.name) {
                    flat.defs.push(Stmt::Let(Def {
                        name: pre(&o.name),
                        expr: e.clone(),
                    }));
                }
            }
            // Encapsulation: this scope reads an instance only through its outputs.
            let own: Vec<&Expr> = self
                .defs
                .iter()
                .flat_map(|s| match s {
                    Stmt::Let(d) => vec![&d.expr],
                    Stmt::Use(o) => o.bind.iter().map(|b| &b.expr).collect(),
                    Stmt::Fsm(f) => f.transitions.iter().map(|t| &t.guard).collect(),
                })
                .chain(self.next.iter().map(|d| &d.expr))
                .collect();
            let reads = own.iter().flat_map(|e| e.vars());
            for n in reads {
                if let Some(x) = n.strip_prefix(&format!("{}__", u.name))
                    && !c.outputs.iter().any(|o| o.name == x)
                {
                    return Err(format!(
                        "reads {n}, but {x} is not an output of {}",
                        u.component
                    ));
                }
            }
        }
        Ok(flat)
    }

    /// Initial state, in the order of `self.state`.
    pub fn init<D: Domain>(&self, d: &mut D) -> Vec<V<D>> {
        self.state
            .iter()
            .map(|s| s.init.eval(d, &Vec::new()))
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
            if bad_range(p.range) {
                bad.push(format!("input {}: bad range {:?}", p.name, p.range));
            }
        }
        for p in &self.params {
            let t = declare(&mut scope, &p.name, Ty::F32, &mut bad);
            scope.push((&p.name, t));
        }
        for s in &self.state {
            if bad_range(s.range) {
                bad.push(format!("state {}: bad range {:?}", s.name, s.range));
            }
            let t = match s.init {
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
        for stmt in &self.defs {
            let Stmt::Let(def) = stmt else {
                bad.push("component instance not flattened".into());
                continue;
            };
            let t = def.expr.ty(&scope).unwrap_or_else(|e| {
                bad.push(format!("{}: {e}", def.name));
                Ty::F32
            });
            let t = declare(&mut scope, &def.name, t, &mut bad);
            scope.push((&def.name, t));
        }
        for p in &self.outputs {
            if bad_range(p.range) {
                bad.push(format!("output {}: bad range {:?}", p.name, p.range));
            }
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

fn bad_range(r: Option<[f64; 2]>) -> bool {
    r.is_some_and(|[lo, hi]| lo.is_nan() || hi.is_nan() || lo > hi)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn port(name: &str) -> Port {
        Port {
            name: name.into(),
            ty: Ty::F32,
            unit: None,
            range: None,
            glitch: false,
        }
    }
    fn def(name: &str, expr: Expr) -> Def {
        Def {
            name: name.into(),
            expr,
        }
    }
    /// `acc`: out = x * gain + total; total accumulates. One param, one state, one internal def.
    fn lib() -> Vec<Component> {
        vec![Component {
            name: "acc".into(),
            compute: Compute {
                inputs: vec![port("x")],
                params: vec![Param {
                    name: "gain".into(),
                    value: 2.0,
                    unit: None,
                }],
                state: vec![StateVar {
                    name: "total".into(),
                    init: num(0.0),
                    unit: None,
                    range: None,
                }],
                defs: vec![
                    def("scaled", var("x") * var("gain")).into(),
                    def("out", var("scaled") + var("total")).into(),
                ],
                outputs: vec![port("out")],
                next: vec![def("total", var("out"))],
            },
        }]
    }
    fn parent(read: &str, bind: Expr) -> Compute {
        let u = |name: &str, bind: Expr| {
            Stmt::Use(Use {
                name: name.into(),
                component: "acc".into(),
                bind: vec![def("x", bind)],
            })
        };
        Compute {
            inputs: vec![port("a")],
            params: vec![],
            state: vec![],
            defs: vec![
                u("p", bind.clone()),
                u("q", bind),
                def("y", var(read) + var("q__out")).into(),
            ],
            outputs: vec![port("y")],
            next: vec![],
        }
    }

    #[test]
    fn two_instances_keep_separate_state() {
        let flat = parent("p__out", var("a")).flatten(&lib()).unwrap();
        assert!(flat.validate().is_empty(), "{:?}", flat.validate());
        let mut state = flat.init(&mut F64);
        let mut ys = Vec::new();
        for a in [1.0, 1.0, 1.0] {
            let (out, next) = flat.step(&mut F64, vec![Val::N(a)], state);
            state = next;
            ys.push(out[0].clone());
        }
        // Each instance: 2, 4, 6. Their sum: 4, 8, 12.
        assert_eq!(ys, [4.0, 8.0, 12.0].map(Val::N));
    }

    #[test]
    fn composite_binding_becomes_a_def_simple_one_is_substituted() {
        let flat = parent("p__out", var("a") + num(1.0))
            .flatten(&lib())
            .unwrap();
        let names: Vec<_> = flat
            .defs
            .iter()
            .map(|s| match s {
                Stmt::Let(d) => d.name.as_str(),
                _ => "not flat",
            })
            .collect();
        assert_eq!(names[0], "p__x");
        let flat = parent("p__out", var("a")).flatten(&lib()).unwrap();
        assert!(
            flat.defs
                .iter()
                .all(|s| !matches!(s, Stmt::Let(d) if d.name == "p__x"))
        );
    }

    #[test]
    fn output_that_passes_an_input_through() {
        let mut l = lib();
        l[0].compute.outputs.push(port("x"));
        let mut c = parent("p__out", var("a"));
        c.defs.push(def("z", var("p__x")).into());
        let flat = c.flatten(&l).unwrap();
        assert!(flat.validate().is_empty(), "{:?}", flat.validate());
        let (_, _) = flat.step(&mut F64, vec![Val::N(3.0)], flat.init(&mut F64));
    }

    #[test]
    fn misuse_is_refused() {
        let e = parent("p__scaled", var("a")).flatten(&lib()).unwrap_err();
        assert!(e.contains("not an output"), "{e}");
        let with = |f: &dyn Fn(&mut Use)| {
            let mut c = parent("p__out", var("a"));
            let Stmt::Use(u) = &mut c.defs[0] else {
                unreachable!()
            };
            f(u);
            c.flatten(&lib()).unwrap_err()
        };
        assert!(with(&|u| u.bind.clear()).contains("not bound"));
        assert!(with(&|u| u.bind.push(def("nope", var("a")))).contains("not an input"));
        assert!(with(&|u| u.component = "missing".into()).contains("no component"));
        let mut rec = lib();
        rec[0].compute.defs.push(Stmt::Use(Use {
            name: "me".into(),
            component: "acc".into(),
            bind: vec![def("x", var("x"))],
        }));
        assert!(
            parent("p__out", var("a"))
                .flatten(&rec)
                .unwrap_err()
                .contains("deep")
        );
    }
}
