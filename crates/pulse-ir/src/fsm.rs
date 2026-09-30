//! State machines as data: named states, an initial state, and transitions in priority order. Checked structurally
//! here, then lowered into plain expressions by `Compute::flatten` (a state variable holding the state's index, a
//! select chain for the next state), so codegen and every proof need nothing new. By construction the machine is
//! deterministic (the first enabled transition wins) and total (no enabled transition: stay).

use crate::expr::{Def, Expr, Param, StateVar, num, select, var};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Transition {
    /// States it leaves from; empty: any state.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub from: Vec<String>,
    pub to: String,
    /// When it fires (boolean). May read inputs, params, state, earlier defs, and state names (their codes).
    pub guard: Expr,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fsm {
    /// The state variable; holds the current state's index into `states`.
    pub state: String,
    /// The def holding the state after this firing's transition.
    pub next: String,
    /// Each state's code is its index; each name is also a param with that code.
    pub states: Vec<String>,
    pub initial: String,
    /// In priority order: the first whose `from` matches and whose guard holds is taken.
    pub transitions: Vec<Transition>,
}

impl Fsm {
    fn code(&self, s: &str) -> Option<f64> {
        self.states.iter().position(|x| x == s).map(|i| i as f64)
    }

    /// Structural checks: known, unique states; every state reachable from the initial one over the transition
    /// graph. (Whether a guard can ever hold is not checked here.)
    pub fn check(&self) -> Vec<String> {
        let mut bad = Vec::new();
        let at = |what: String| format!("{}: {what}", self.state);
        if self.states.is_empty() {
            bad.push(at("no states".into()));
        }
        for (i, s) in self.states.iter().enumerate() {
            if !crate::is_ident(s) || self.states[..i].contains(s) {
                bad.push(at(format!("state {s:?} is not a unique identifier")));
            }
        }
        let known = |s: &String, bad: &mut Vec<String>| {
            if !self.states.contains(s) {
                bad.push(at(format!("unknown state {s:?}")));
            }
        };
        known(&self.initial, &mut bad);
        for t in &self.transitions {
            t.from.iter().for_each(|s| known(s, &mut bad));
            known(&t.to, &mut bad);
        }
        if !bad.is_empty() {
            return bad;
        }
        let mut seen = vec![self.initial.clone()];
        let mut i = 0;
        while i < seen.len() {
            for t in &self.transitions {
                if (t.from.is_empty() || t.from.contains(&seen[i])) && !seen.contains(&t.to) {
                    seen.push(t.to.clone());
                }
            }
            i += 1;
        }
        for s in self.states.iter().filter(|s| !seen.contains(s)) {
            bad.push(at(format!(
                "state {s} is unreachable from {}",
                self.initial
            )));
        }
        bad
    }

    /// States no transition leaves (once entered, never left).
    pub fn absorbing(&self) -> Vec<&str> {
        let leaves = |s: &String| {
            self.transitions
                .iter()
                .any(|t| &t.to != s && (t.from.is_empty() || t.from.contains(s)))
        };
        self.states
            .iter()
            .filter(|s| !leaves(s))
            .map(|s| s.as_str())
            .collect()
    }

    /// The lowered form: params (state names), the state variable, the next-state def and its update.
    pub fn lower(&self) -> (Vec<Param>, StateVar, Def, Def) {
        let params = self
            .states
            .iter()
            .enumerate()
            .map(|(i, s)| Param {
                name: s.clone(),
                value: i as f64,
                unit: Some("1".into()),
            })
            .collect();
        let state = StateVar {
            name: self.state.clone(),
            init: num(self.code(&self.initial).unwrap_or(0.0)),
            unit: Some("1".into()),
            range: Some([0.0, self.states.len().saturating_sub(1) as f64]),
        };
        let mut next = var(&self.state);
        for t in self.transitions.iter().rev() {
            let from = t
                .from
                .iter()
                .map(|s| var(&self.state).eq(num(self.code(s).unwrap_or(0.0))))
                .reduce(Expr::or);
            let cond = match from {
                Some(f) => f.and(t.guard.clone()),
                None => t.guard.clone(),
            };
            next = select(cond, num(self.code(&t.to).unwrap_or(0.0)), next);
        }
        let def = Def {
            name: self.next.clone(),
            expr: next,
        };
        let update = Def {
            name: self.state.clone(),
            expr: var(&self.next),
        };
        (params, state, def, update)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(from: &[&str], to: &str) -> Transition {
        Transition {
            from: from.iter().map(|s| s.to_string()).collect(),
            to: to.into(),
            guard: Expr::Bool(true),
        }
    }
    fn fsm(transitions: Vec<Transition>) -> Fsm {
        Fsm {
            state: "mode".into(),
            next: "mode_next".into(),
            states: ["a", "b", "c"].map(String::from).to_vec(),
            initial: "a".into(),
            transitions,
        }
    }

    #[test]
    fn structure_is_checked() {
        assert!(fsm(vec![t(&["a"], "b"), t(&[], "c")]).check().is_empty());
        let e = fsm(vec![t(&["a"], "b")]).check();
        assert!(e.iter().any(|m| m.contains("c is unreachable")), "{e:?}");
        assert!(fsm(vec![t(&["a"], "zz")]).check()[0].contains("unknown state"));
        let mut dup = fsm(vec![]);
        dup.states.push("a".into());
        assert!(dup.check().iter().any(|m| m.contains("not a unique")));
    }

    #[test]
    fn absorbing_states_are_reported() {
        let f = fsm(vec![t(&["a"], "b"), t(&["b"], "a"), t(&[], "c")]);
        assert_eq!(f.absorbing(), ["c"]);
    }
}
