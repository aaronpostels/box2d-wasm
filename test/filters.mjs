// Filters checks in the Rust smoke module (crates/box2d-smoke/src/lib.rs, fn filters): shape filter round trip, and
// category, mask and group decide which balls fall through which floors. run_filters returns 0, or the code of the
// first failing check (80 to 86). Needs no dist build.
// Build the smoke wasm first: cd crates/box2d-smoke && cargo build --target wasm32-unknown-unknown --release
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const wasm = new URL("../crates/box2d-smoke/target/wasm32-unknown-unknown/release/box2d_smoke.wasm", import.meta.url);
const module = new WebAssembly.Module(readFileSync(wasm));
assert.equal(WebAssembly.Module.imports(module).length, 0, "smoke module should import nothing");
const { run_filters } = new WebAssembly.Instance(module, {}).exports;
assert.equal(typeof run_filters, "function", "smoke module should export run_filters");
const code = run_filters();
assert.equal(code, 0, `filters check failed with code ${code}`);
console.log("ok filters: round trip, category and mask, negative group falls through, balls rest on the right floor");
