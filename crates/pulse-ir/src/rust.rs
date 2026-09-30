//! Codegen: a block's `Compute` as a `no_std`, allocation-free, panic-free Rust struct. It is `Compute::step` run in
//! a domain whose values are Rust source, so the generated code performs the same f32 operations, in the same order,
//! as the `expr::F32` interpreter.

use crate::expr::{Cmp, Compute, Domain, Op, Ty, V, Val};
use std::fmt::Write;

struct Emit {
    lets: String,
}

fn lit(x: f32) -> String {
    match x {
        f32::INFINITY => "f32::INFINITY".into(),
        f32::NEG_INFINITY => "f32::NEG_INFINITY".into(),
        _ if x.is_nan() => "f32::NAN".into(),
        // Debug prints the shortest string that round-trips, so the literal is exact.
        _ if x.is_sign_negative() => format!("({x:?}_f32)"),
        _ => format!("{x:?}_f32"),
    }
}

impl Domain for Emit {
    type N = String;
    type B = String;
    fn num(&mut self, x: f64) -> String {
        lit(x as f32)
    }
    fn boolean(&mut self, b: bool) -> String {
        b.to_string()
    }
    fn neg(&mut self, a: String) -> String {
        format!("(-{a})")
    }
    fn op(&mut self, op: Op, a: String, b: String) -> String {
        match op {
            Op::Add => format!("({a} + {b})"),
            Op::Sub => format!("({a} - {b})"),
            Op::Mul => format!("({a} * {b})"),
            Op::Div => format!("({a} / {b})"),
            Op::Max => format!("{a}.max({b})"),
            Op::Min => format!("{a}.min({b})"),
        }
    }
    fn cmp(&mut self, cmp: Cmp, a: String, b: String) -> String {
        let o = match cmp {
            Cmp::Lt => "<",
            Cmp::Le => "<=",
            Cmp::Gt => ">",
            Cmp::Ge => ">=",
            Cmp::Eq => "==",
        };
        format!("({a} {o} {b})")
    }
    fn logic(&mut self, and: bool, a: String, b: String) -> String {
        format!("({a} {} {b})", if and { "&&" } else { "||" })
    }
    fn not(&mut self, a: String) -> String {
        format!("(!{a})")
    }
    fn is_finite(&mut self, a: String) -> String {
        format!("{a}.is_finite()")
    }
    fn select(&mut self, c: String, a: V<Self>, b: V<Self>) -> V<Self> {
        let s = |v: V<Self>| match v {
            Val::N(x) | Val::B(x) => x,
        };
        let e = format!("(if {c} {{ {} }} else {{ {} }})", s(a.clone()), s(b));
        match a {
            Val::N(_) => Val::N(e),
            Val::B(_) => Val::B(e),
        }
    }
    fn bind(&mut self, next: bool, name: &str, v: V<Self>) -> V<Self> {
        let local = format!("{}_{name}", if next { "n" } else { "v" });
        let (Val::N(e) | Val::B(e)) = &v;
        writeln!(self.lets, "        let {local} = {e};").unwrap();
        match v {
            Val::N(_) => Val::N(local),
            Val::B(_) => Val::B(local),
        }
    }
}

fn rust_ty(t: Ty) -> &'static str {
    match t {
        Ty::F32 => "f32",
        Ty::Bool => "bool",
        Ty::U8 => "u8",
    }
}

/// `pub struct {name}` with `new()` and `step(inputs) -> (outputs)`. `c` must have passed `Compute::validate`.
pub fn emit(name: &str, c: &Compute) -> String {
    let mut d = Emit {
        lets: String::new(),
    };
    let inputs = c
        .inputs
        .iter()
        .map(|p| match p.ty {
            Ty::Bool => Val::B(format!("v_{}", p.name)),
            _ => Val::N(format!("v_{}", p.name)),
        })
        .collect();
    let init = c.init(&mut d);
    let state = init
        .iter()
        .zip(&c.state)
        .map(|(v, s)| match v {
            Val::N(_) => Val::N(format!("self.{}", s.name)),
            Val::B(_) => Val::B(format!("self.{}", s.name)),
        })
        .collect();
    let (outs, next) = c.step(&mut d, inputs, state);

    let mut s = String::new();
    let fields: Vec<(&str, &str, String)> = c
        .state
        .iter()
        .zip(&init)
        .map(|(s, v)| match v {
            Val::N(x) => (s.name.as_str(), "f32", x.clone()),
            Val::B(x) => (s.name.as_str(), "bool", x.clone()),
        })
        .collect();
    writeln!(s, "pub struct {name} {{").unwrap();
    for (f, t, _) in &fields {
        writeln!(s, "    pub {f}: {t},").unwrap();
    }
    writeln!(s, "}}\n\nimpl Default for {name} {{\n    fn default() -> Self {{\n        Self::new()\n    }}\n}}\n").unwrap();
    writeln!(
        s,
        "impl {name} {{\n    pub const fn new() -> Self {{\n        Self {{"
    )
    .unwrap();
    for (f, _, init) in &fields {
        writeln!(s, "            {f}: {init},").unwrap();
    }
    let args: Vec<String> = c
        .inputs
        .iter()
        .map(|p| format!("v_{}: {}", p.name, rust_ty(p.ty)))
        .collect();
    let ret: Vec<&str> = c.outputs.iter().map(|p| rust_ty(p.ty)).collect();
    writeln!(s, "        }}\n    }}\n").unwrap();
    writeln!(
        s,
        "    pub fn step(&mut self, {}) -> ({},) {{",
        args.join(", "),
        ret.join(", ")
    )
    .unwrap();
    for p in c.inputs.iter().filter(|p| p.ty == Ty::U8) {
        writeln!(s, "        let v_{0} = v_{0} as f32;", p.name).unwrap();
    }
    s.push_str(&d.lets);
    // Outputs are taken before the state is overwritten: an output naming a state reads its old value.
    let outs: Vec<String> = outs
        .into_iter()
        .map(|v| match v {
            Val::N(e) | Val::B(e) => e,
        })
        .collect();
    writeln!(s, "        let out = ({},);", outs.join(", ")).unwrap();
    for (st, v) in c.state.iter().zip(next) {
        let (Val::N(e) | Val::B(e)) = v;
        if e != format!("self.{}", st.name) {
            writeln!(s, "        self.{} = {e};", st.name).unwrap();
        }
    }
    writeln!(s, "        out\n    }}\n}}").unwrap();
    s
}
