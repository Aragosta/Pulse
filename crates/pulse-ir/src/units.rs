//! Unit checking: every numeric expression has a unit (products of base symbols with integer powers, e.g.
//! `V/(A*s)`); `+ - max min select` and comparisons need equal units, `* /` combine them. A bare literal adopts
//! whatever its context needs, so `error > 0` is fine and `amps + volts` is not. Symbols are independent (no
//! `V = W/A` conversions): a model must use one spelling consistently.

use crate::expr::{Component, Compute, Expr, Stmt, Ty};
use std::collections::BTreeMap;

/// Base symbol -> power. Empty: dimensionless.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Unit(BTreeMap<String, i32>);

impl Unit {
    fn combine(&self, o: &Unit, sign: i32) -> Unit {
        let mut m = self.0.clone();
        for (k, p) in &o.0 {
            *m.entry(k.clone()).or_insert(0) += sign * p;
        }
        m.retain(|_, p| *p != 0);
        Unit(m)
    }
}

impl std::fmt::Display for Unit {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        if self.0.is_empty() {
            return write!(f, "1");
        }
        let parts: Vec<String> = self
            .0
            .iter()
            .map(|(k, p)| {
                if *p == 1 {
                    k.clone()
                } else {
                    format!("{k}^{p}")
                }
            })
            .collect();
        write!(f, "{}", parts.join("*"))
    }
}

/// `A`, `V/A`, `V/(A*s)`, `V*s/A`, `rad/s^2`, `1`.
pub fn parse(s: &str) -> Result<Unit, String> {
    let toks: Vec<char> = s.chars().filter(|c| !c.is_whitespace()).collect();
    let mut i = 0;
    let u = product(&toks, &mut i)?;
    if i != toks.len() {
        return Err(format!("unit {s:?}: unexpected {:?}", toks[i]));
    }
    Ok(u)
}
fn product(t: &[char], i: &mut usize) -> Result<Unit, String> {
    let mut u = factor(t, i)?;
    while *i < t.len() && (t[*i] == '*' || t[*i] == '/') {
        let sign = if t[*i] == '*' { 1 } else { -1 };
        *i += 1;
        u = u.combine(&factor(t, i)?, sign);
    }
    Ok(u)
}
fn factor(t: &[char], i: &mut usize) -> Result<Unit, String> {
    let base = if t.get(*i) == Some(&'(') {
        *i += 1;
        let u = product(t, i)?;
        if t.get(*i) != Some(&')') {
            return Err("unit: missing ')'".into());
        }
        *i += 1;
        u
    } else {
        let start = *i;
        while *i < t.len() && (t[*i].is_alphanumeric() || t[*i] == '_') {
            *i += 1;
        }
        let sym: String = t[start..*i].iter().collect();
        match sym.as_str() {
            "" => return Err("unit: expected a symbol".into()),
            "1" => Unit(BTreeMap::new()),
            _ if sym.starts_with(|c: char| c.is_ascii_digit()) => {
                return Err(format!("unit: bad symbol {sym:?}"));
            }
            _ => Unit(BTreeMap::from([(sym, 1)])),
        }
    };
    if t.get(*i) != Some(&'^') {
        return Ok(base);
    }
    *i += 1;
    let start = *i;
    if t.get(*i) == Some(&'-') {
        *i += 1;
    }
    while *i < t.len() && t[*i].is_ascii_digit() {
        *i += 1;
    }
    let p: i32 = t[start..*i]
        .iter()
        .collect::<String>()
        .parse()
        .map_err(|_| "unit: bad power")?;
    Ok(Unit(
        base.0
            .into_iter()
            .map(|(k, q)| (k, q * p))
            .filter(|(_, q)| *q != 0)
            .collect(),
    ))
}

/// What an expression's unit is known to be.
#[derive(Clone, Debug, PartialEq)]
enum U {
    /// A bare literal: adopts its context's unit (dimensionless in `*` and `/`).
    Lit,
    /// No unit declared on something it reads: not checked.
    Unknown,
    Known(Unit),
    Bool,
}

fn declared(unit: &Option<String>, ty: Ty) -> Result<U, String> {
    match (ty, unit) {
        (Ty::Bool, _) => Ok(U::Bool),
        (_, None) => Ok(U::Unknown),
        (_, Some(s)) => parse(s).map(U::Known),
    }
}

/// The common unit of two operands that must agree.
fn same(a: U, b: U, what: &Expr) -> Result<U, String> {
    match (a, b) {
        (U::Known(x), U::Known(y)) if x != y => Err(format!("{x} vs {y} in {what:?}")),
        (U::Known(x), _) | (_, U::Known(x)) => Ok(U::Known(x)),
        (U::Unknown, _) | (_, U::Unknown) => Ok(U::Unknown),
        (a, _) => Ok(a),
    }
}

fn unit(e: &Expr, scope: &[(String, U)]) -> Result<U, String> {
    use Expr::*;
    let u = |e: &Expr| unit(e, scope);
    Ok(match e {
        Num(_) => U::Lit,
        Bool(_) => U::Bool,
        Var(n) => scope
            .iter()
            .rev()
            .find(|(s, _)| s == n)
            .map_or(U::Unknown, |(_, u)| u.clone()),
        Neg(a) => u(a)?,
        Add(x, y) | Sub(x, y) | Max(x, y) | Min(x, y) => same(u(x)?, u(y)?, e)?,
        Mul(x, y) | Div(x, y) => {
            let sign = if matches!(e, Mul(..)) { 1 } else { -1 };
            let one = Unit(BTreeMap::new());
            match (u(x)?, u(y)?) {
                (U::Unknown, _) | (_, U::Unknown) => U::Unknown,
                (U::Lit, U::Lit) => U::Lit,
                (U::Known(a), U::Lit) => U::Known(a),
                (U::Lit, U::Known(b)) => U::Known(one.combine(&b, sign)),
                (U::Known(a), U::Known(b)) => U::Known(a.combine(&b, sign)),
                _ => return Err(format!("boolean in arithmetic: {e:?}")),
            }
        }
        Lt(x, y) | Le(x, y) | Gt(x, y) | Ge(x, y) | Eq(x, y) => {
            same(u(x)?, u(y)?, e)?;
            U::Bool
        }
        And(..) | Or(..) | Not(_) | IsFinite(_) => U::Bool,
        Select(_, x, y) => same(u(x)?, u(y)?, e)?,
    })
}

/// Unit errors in `c` (not flattened: each component is checked once on its own, and each instance's bindings
/// against the component's declared input units).
pub fn check(c: &Compute, lib: &[Component]) -> Vec<String> {
    let mut bad = Vec::new();
    let mut scope: Vec<(String, U)> = Vec::new();
    let mut decl = |name: &str, unit: &Option<String>, ty: Ty, scope: &mut Vec<(String, U)>| {
        let u = declared(unit, ty).unwrap_or_else(|e| {
            bad.push(format!("{name}: {e}"));
            U::Unknown
        });
        scope.push((name.to_string(), u));
    };
    for p in &c.inputs {
        decl(&p.name, &p.unit, p.ty, &mut scope);
    }
    for p in &c.params {
        decl(&p.name, &p.unit, Ty::F32, &mut scope);
    }
    for s in &c.state {
        let ty = if matches!(s.init, Expr::Bool(_)) {
            Ty::Bool
        } else {
            Ty::F32
        };
        decl(&s.name, &s.unit, ty, &mut scope);
    }
    let agree = |name: &str, got: Result<U, String>, want: U, bad: &mut Vec<String>| match got {
        Err(e) => bad.push(format!("{name}: {e}")),
        Ok(got) => {
            if let (U::Known(g), U::Known(w)) = (&got, &want)
                && g != w
            {
                bad.push(format!("{name}: computes {g}, declared {w}"));
            }
        }
    };
    for stmt in &c.defs {
        match stmt {
            Stmt::Let(d) => {
                let u = unit(&d.expr, &scope).unwrap_or_else(|e| {
                    bad.push(format!("{}: {e}", d.name));
                    U::Unknown
                });
                scope.push((d.name.clone(), u));
            }
            Stmt::Use(inst) => {
                let Some(comp) = lib.iter().find(|x| x.name == inst.component) else {
                    continue; // reported by flatten
                };
                for p in &comp.compute.inputs {
                    if let Some(b) = inst.bind.iter().find(|b| b.name == p.name) {
                        let want = declared(&p.unit, p.ty).unwrap_or(U::Unknown);
                        agree(
                            &format!("{}.{}", inst.name, p.name),
                            unit(&b.expr, &scope),
                            want,
                            &mut bad,
                        );
                    }
                }
                for o in &comp.compute.outputs {
                    let u = declared(&o.unit, o.ty).unwrap_or(U::Unknown);
                    scope.push((format!("{}__{}", inst.name, o.name), u));
                }
            }
        }
    }
    for p in &c.outputs {
        let want = declared(&p.unit, p.ty).unwrap_or(U::Unknown);
        agree(
            &format!("output {}", p.name),
            unit(&Expr::Var(p.name.clone()), &scope),
            want,
            &mut bad,
        );
    }
    for n in &c.next {
        let want = scope
            .iter()
            .find(|(s, _)| *s == n.name)
            .map_or(U::Unknown, |(_, u)| u.clone());
        agree(
            &format!("next {}", n.name),
            unit(&n.expr, &scope),
            want,
            &mut bad,
        );
    }
    bad
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::expr::{Def, Port, num, var};

    #[test]
    fn parses_and_normalises() {
        assert_eq!(parse("V/(A*s)").unwrap(), parse("V*A^-1/s").unwrap());
        assert_eq!(parse("V*s/A").unwrap().to_string(), "A^-1*V*s");
        assert_eq!(parse("A/A").unwrap(), parse("1").unwrap());
        assert!(parse("V/(A").is_err() && parse("2V").is_err() && parse("V^x").is_err());
    }

    fn block(defs: Vec<(&str, Expr)>, out_unit: &str) -> Compute {
        let port = |n: &str, u: &str| Port {
            name: n.into(),
            ty: Ty::F32,
            unit: Some(u.into()),
            range: None,
            glitch: false,
        };
        Compute {
            inputs: vec![port("i", "A"), port("v", "V"), port("t", "s")],
            params: vec![],
            state: vec![],
            defs: defs
                .into_iter()
                .map(|(n, e)| {
                    Def {
                        name: n.into(),
                        expr: e,
                    }
                    .into()
                })
                .collect(),
            outputs: vec![port("out", out_unit)],
            next: vec![],
        }
    }

    #[test]
    fn mismatches_are_refused_and_literals_adopt() {
        let ok = |d: Vec<(&str, Expr)>, u| check(&block(d, u), &[]);
        assert!(ok(vec![("out", var("v") / var("i"))], "V/A").is_empty());
        assert!(ok(vec![("out", var("i") * var("t") + num(1.0))], "A*s").is_empty());
        assert!(ok(vec![("out", num(2.0) / var("t"))], "1/s").is_empty());
        assert!(ok(vec![("out", var("i").max(num(0.0)))], "A").is_empty());
        assert!(!ok(vec![("out", var("i") + var("v"))], "A").is_empty());
        assert!(!ok(vec![("out", var("i"))], "V").is_empty());
        assert!(!ok(vec![("c", var("i").gt(var("v"))), ("out", var("i"))], "A").is_empty());
    }
}
