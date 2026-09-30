//! `pulse`: the checks and codegen as a command, for people, CI and agents.
//!
//!   pulse check <model.json|-> [--json]   every check; exit 0 proved, 1 violations, 2 usage or I/O
//!   pulse gen <model.json|->              the firmware step as Rust, only for an IR that checks
//!   pulse mcp                             the same two as MCP tools over stdio
//!
//! No logic of its own: `check` is `evidence::evidence`, `gen` is `rust::firmware`, the same
//! calls the example and its tests make.

use pulse_ir::{Ir, Violation, evidence, rust};
use serde_json::{Value, json};
use std::io::{BufRead, Read, Write};
use std::process::exit;

const USAGE: &str =
    "usage: pulse check <model.json|-> [--json] | pulse gen <model.json|-> | pulse mcp";

/// Parse, then every check. A parse error is a violation too, so an agent gets it in the same shape.
fn check(ir: Result<Ir, String>) -> Result<(Ir, evidence::Evidence), Vec<Violation>> {
    let ir = ir.map_err(|e| {
        vec![Violation {
            check: "ir",
            code: "IR-PARSE",
            msg: e,
        }]
    })?;
    let ev = evidence::evidence(&ir)?;
    Ok((ir, ev))
}

fn report(r: &Result<(Ir, evidence::Evidence), Vec<Violation>>) -> Value {
    match r {
        Ok((_, ev)) => json!({"ok": true, "violations": [], "evidence": ev}),
        Err(vs) => json!({"ok": false, "violations": vs, "evidence": null}),
    }
}

fn human(r: &Result<(Ir, evidence::Evidence), Vec<Violation>>) -> String {
    let mut s = String::new();
    match r {
        Ok((_, ev)) => {
            s += &format!("PROVED for IR {}\n", ev.ir_hash);
            for (head, items) in [
                ("", &ev.proved),
                ("ASSUMING\n", &ev.assumed),
                ("NOT PROVED\n", &ev.not_proved),
            ] {
                s += head;
                for i in items {
                    s += &format!("  - {i}\n");
                }
            }
        }
        Err(vs) => {
            s += "REFUSED\n";
            for v in vs {
                s += &format!("  [FAIL] {} ({}): {}\n", v.check, v.code, v.msg);
            }
        }
    }
    s
}

fn read_model(path: &str) -> Result<String, String> {
    if path == "-" {
        let mut s = String::new();
        std::io::stdin()
            .read_to_string(&mut s)
            .map_err(|e| format!("stdin: {e}"))?;
        Ok(s)
    } else {
        std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))
    }
}

fn parse(src: &str) -> Result<Ir, String> {
    serde_json::from_str(src).map_err(|e| e.to_string())
}

// ---- MCP (2025-11-25): newline-delimited JSON-RPC 2.0 on stdin/stdout ------------------------------------------------

fn tools() -> Value {
    let input = json!({
        "type": "object",
        "properties": {"ir": {
            "type": "object",
            "description": "A Pulse IR (version 4) as JSON: blocks, edges, components. See SEMANTICS.md."
        }},
        "required": ["ir"]
    });
    json!([
        {
            "name": "check",
            "description": "Run every Pulse check (validation, units, Class 1 timing, Class 3 invariants and ranges) \
                on an IR. Returns violations with stable codes, or the evidence: what is proved, assumed and not \
                proved, bound to the IR's hash.",
            "inputSchema": input,
        },
        {
            "name": "generate",
            "description": "Generate the no_std Rust firmware step for an IR. Refused, with the same violations as \
                `check`, unless every check passes.",
            "inputSchema": input,
        },
    ])
}

fn call_tool(params: &Value) -> Value {
    let name = params["name"].as_str().unwrap_or("");
    let ir = match params["arguments"].get("ir") {
        Some(v) => serde_json::from_value::<Ir>(v.clone()).map_err(|e| e.to_string()),
        None => Err("missing argument `ir`".into()),
    };
    let r = check(ir);
    let (structured, text, is_error) = match (name, &r) {
        ("check", _) => (report(&r), human(&r), r.is_err()),
        ("generate", Ok((ir, ev))) => {
            let src = rust::firmware(ir, &ev.ir_hash);
            (
                json!({"ok": true, "ir_hash": ev.ir_hash, "rust": src}),
                src,
                false,
            )
        }
        ("generate", Err(_)) => (report(&r), human(&r), true),
        _ => {
            return json!({
                "content": [{"type": "text", "text": format!("unknown tool {name:?}")}],
                "isError": true,
            });
        }
    };
    json!({
        "content": [{"type": "text", "text": text}],
        "structuredContent": structured,
        "isError": is_error,
    })
}

fn mcp() {
    let mut out = std::io::stdout().lock();
    for line in std::io::stdin().lock().lines() {
        let Ok(line) = line else { break };
        if line.trim().is_empty() {
            continue;
        }
        let msg: Value = match serde_json::from_str(&line) {
            Ok(m) => m,
            Err(e) => {
                let err = json!({"jsonrpc": "2.0", "id": null,
                    "error": {"code": -32700, "message": format!("parse error: {e}")}});
                writeln!(out, "{err}").ok();
                continue;
            }
        };
        // A message without an id is a notification: never answered.
        let Some(id) = msg.get("id").cloned() else {
            continue;
        };
        let params = &msg["params"];
        let result = match msg["method"].as_str().unwrap_or("") {
            "initialize" => Ok(json!({
                "protocolVersion": params["protocolVersion"].as_str().unwrap_or("2025-11-25"),
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "pulse", "version": env!("CARGO_PKG_VERSION")},
            })),
            "ping" => Ok(json!({})),
            "tools/list" => Ok(json!({"tools": tools()})),
            "tools/call" => Ok(call_tool(params)),
            m => Err(json!({"code": -32601, "message": format!("method not found: {m}")})),
        };
        let resp = match result {
            Ok(r) => json!({"jsonrpc": "2.0", "id": id, "result": r}),
            Err(e) => json!({"jsonrpc": "2.0", "id": id, "error": e}),
        };
        writeln!(out, "{resp}").ok();
        out.flush().ok();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args: Vec<&str> = args.iter().map(String::as_str).collect();
    let load = |path: &str| {
        read_model(path).unwrap_or_else(|e| {
            eprintln!("pulse: {e}");
            exit(2)
        })
    };
    match args.as_slice() {
        ["mcp"] => mcp(),
        ["check", path, rest @ ..] if rest.is_empty() || rest == ["--json"] => {
            let r = check(parse(&load(path)));
            if rest.is_empty() {
                print!("{}", human(&r));
            } else {
                println!("{}", report(&r));
            }
            exit(if r.is_ok() { 0 } else { 1 });
        }
        ["gen", path] => match check(parse(&load(path))) {
            Ok((ir, ev)) => print!("{}", rust::firmware(&ir, &ev.ir_hash)),
            Err(vs) => {
                eprint!("{}", human(&Err(vs)));
                exit(1);
            }
        },
        _ => {
            eprintln!("{USAGE}");
            exit(2);
        }
    }
}
