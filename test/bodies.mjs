// Bodies checks in the Rust smoke module (crates/box2d-smoke/src/lib.rs, fn bodies): linear damping, gravity scale,
// fixed rotation, mass data, user data, enable and disable, batch impulses. run_bodies returns 0, or the code of the
// first failing check (10 to 12, 20 to 22, 30 to 35, 40 to 42, 50 to 52, 60 to 63, 70). Needs no dist build.
// Build the smoke wasm first: cd crates/box2d-smoke && cargo build --target wasm32-unknown-unknown --release
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const wasm = new URL("../crates/box2d-smoke/target/wasm32-unknown-unknown/release/box2d_smoke.wasm", import.meta.url);
const module = new WebAssembly.Module(readFileSync(wasm));
assert.equal(WebAssembly.Module.imports(module).length, 0, "smoke module should import nothing");
const { run_bodies } = new WebAssembly.Instance(module, {}).exports;
assert.equal(typeof run_bodies, "function", "smoke module should export run_bodies");
const code = run_bodies();
assert.equal(code, 0, `bodies check failed with code ${code}`);
console.log("ok bodies: damping, gravity scale, fixed rotation, mass, user data, enable, batch impulses");
