//! The `pulse` binary as an agent or CI uses it: `check` (human or `--json`), `gen`, and `mcp` over stdio. Every
//! test runs the real executable on the single-joint IR JSON that `examples/single_joint` keeps current.

use serde_json::{Value, json};
use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Output, Stdio};

const PULSE: &str = env!("CARGO_BIN_EXE_pulse");
const MODEL: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../examples/single_joint/single_joint.ir.json"
);
/// The hash the single-joint example reports for the same IR (built in Rust, not parsed from JSON).
const HASH: &str = "fnv1a64:1280b212dc18e23e";

fn model() -> Value {
    serde_json::from_str(&std::fs::read_to_string(MODEL).unwrap()).unwrap()
}

/// The model with the current PID's integrator invariant shrunk to +-1 V, which the loop cannot keep.
fn unprovable() -> Value {
    let mut ir = model();
    let pid = ir["components"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find(|c| c["name"] == "current_pid")
        .unwrap();
    for s in pid["compute"]["state"].as_array_mut().unwrap() {
        if s["name"] == "integral" {
            s["range"] = json!([-1.0, 1.0]);
        }
    }
    ir
}

fn run(args: &[&str], stdin: &str) -> Output {
    let mut p = Command::new(PULSE)
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    p.stdin.take().unwrap().write_all(stdin.as_bytes()).unwrap();
    p.wait_with_output().unwrap()
}

fn json_out(o: &Output) -> Value {
    serde_json::from_slice(&o.stdout).unwrap_or_else(|e| {
        panic!(
            "{e}: {}\n{}",
            String::from_utf8_lossy(&o.stdout),
            String::from_utf8_lossy(&o.stderr)
        )
    })
}

fn codes(report: &Value) -> Vec<String> {
    report["violations"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v["code"].as_str().unwrap().to_string())
        .collect()
}

#[test]
fn check_accepts_the_single_joint_and_names_its_hash() {
    let o = run(&["check", MODEL, "--json"], "");
    assert_eq!(o.status.code(), Some(0));
    let r = json_out(&o);
    assert_eq!(r["ok"], true);
    assert_eq!(r["evidence"]["ir_hash"], HASH);
    assert!(r["evidence"]["assumed"].as_array().unwrap().len() > 3);

    let human = run(&["check", MODEL], "");
    assert_eq!(human.status.code(), Some(0));
    let text = String::from_utf8(human.stdout).unwrap();
    assert!(text.contains(HASH) && text.contains("NOT PROVED"), "{text}");
}

#[test]
fn check_reports_violations_by_code() {
    let o = run(&["check", "-", "--json"], &unprovable().to_string());
    assert_eq!(o.status.code(), Some(1));
    let r = json_out(&o);
    assert_eq!(r["ok"], false);
    assert!(codes(&r).contains(&"C3-INVARIANT".into()), "{r}");

    let mut old = model();
    old["version"] = json!(3);
    let r = json_out(&run(&["check", "-", "--json"], &old.to_string()));
    assert!(codes(&r).contains(&"IR-VERSION".into()), "{r}");

    let human = run(&["check", "-"], &unprovable().to_string());
    assert_eq!(human.status.code(), Some(1));
    assert!(
        String::from_utf8(human.stdout)
            .unwrap()
            .contains("C3-INVARIANT")
    );
}

#[test]
fn check_refuses_malformed_json_with_a_location() {
    let o = run(
        &["check", "-", "--json"],
        "{\"version\": 4,\n  \"blocks\": oops }",
    );
    assert_eq!(o.status.code(), Some(1));
    let r = json_out(&o);
    assert_eq!(codes(&r), ["IR-PARSE"]);
    assert!(
        r["violations"][0]["msg"]
            .as_str()
            .unwrap()
            .contains("line 2"),
        "{r}"
    );
}

#[test]
fn usage_errors_exit_2() {
    assert_eq!(run(&[], "").status.code(), Some(2));
    assert_eq!(run(&["frobnicate"], "").status.code(), Some(2));
    assert_eq!(
        run(&["check", "/no/such/model.json"], "").status.code(),
        Some(2)
    );
}

/// What `pulse gen` prints is, byte for byte, the firmware the example ships.
#[test]
fn gen_is_the_shipped_firmware() {
    let o = run(&["gen", MODEL], "");
    assert_eq!(o.status.code(), Some(0));
    let shipped = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../pulse-joint/src/generated.rs"
    ))
    .unwrap();
    let out = String::from_utf8(o.stdout).unwrap();
    assert!(out.contains(HASH), "header names the IR");
    assert_eq!(out, shipped);
}

#[test]
fn gen_refuses_what_check_refuses() {
    let o = run(&["gen", "-"], &unprovable().to_string());
    assert_eq!(o.status.code(), Some(1));
    assert!(o.stdout.is_empty(), "no code for an unproved IR");
    assert!(
        String::from_utf8(o.stderr)
            .unwrap()
            .contains("C3-INVARIANT")
    );
}

/// One MCP session over stdio (newline-delimited JSON-RPC 2.0), as an agent host drives it.
#[test]
fn mcp_session() {
    let mut p = Command::new(PULSE)
        .arg("mcp")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut tx = p.stdin.take().unwrap();
    let mut rx = BufReader::new(p.stdout.take().unwrap());
    let mut call = |msg: Value| -> Option<Value> {
        writeln!(tx, "{msg}").unwrap();
        msg.get("id")?;
        let mut line = String::new();
        rx.read_line(&mut line).unwrap();
        Some(serde_json::from_str(&line).unwrap())
    };
    let req = |id: u32, method: &str, params: Value| json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params});

    let init = call(req(
        1,
        "initialize",
        json!({"protocolVersion": "2025-11-25", "capabilities": {}, "clientInfo": {"name": "t", "version": "0"}}),
    ))
    .unwrap();
    assert_eq!(init["result"]["protocolVersion"], "2025-11-25");
    assert_eq!(init["result"]["serverInfo"]["name"], "pulse");
    assert!(init["result"]["capabilities"]["tools"].is_object());
    assert!(call(json!({"jsonrpc": "2.0", "method": "notifications/initialized"})).is_none());

    let tools = call(req(2, "tools/list", json!({}))).unwrap();
    let names: Vec<&str> = tools["result"]["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    assert_eq!(names, ["check", "generate"]);

    let ok = call(req(
        3,
        "tools/call",
        json!({"name": "check", "arguments": {"ir": model()}}),
    ))
    .unwrap();
    assert_eq!(ok["result"]["isError"], false);
    assert_eq!(ok["result"]["structuredContent"]["ok"], true);
    assert_eq!(
        ok["result"]["structuredContent"]["evidence"]["ir_hash"],
        HASH
    );
    assert_eq!(ok["result"]["content"][0]["type"], "text");

    let bad = call(req(
        4,
        "tools/call",
        json!({"name": "check", "arguments": {"ir": unprovable()}}),
    ))
    .unwrap();
    assert_eq!(bad["result"]["isError"], true);
    assert!(codes(&bad["result"]["structuredContent"]).contains(&"C3-INVARIANT".into()));

    let gen_ = call(req(
        5,
        "tools/call",
        json!({"name": "generate", "arguments": {"ir": model()}}),
    ))
    .unwrap();
    assert_eq!(gen_["result"]["isError"], false);
    assert!(
        gen_["result"]["structuredContent"]["rust"]
            .as_str()
            .unwrap()
            .contains("pub fn step(")
    );

    let missing = call(req(
        6,
        "tools/call",
        json!({"name": "check", "arguments": {}}),
    ))
    .unwrap();
    assert_eq!(missing["result"]["isError"], true);
    let unknown = call(req(7, "resources/list", json!({}))).unwrap();
    assert_eq!(unknown["error"]["code"], -32601);
    assert_eq!(
        call(req(8, "ping", json!({}))).unwrap()["result"],
        json!({})
    );

    drop(tx);
    assert!(p.wait().unwrap().success(), "exits cleanly on EOF");
}
