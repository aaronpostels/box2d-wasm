// Runs the shapes checks of the Rust smoke module (crates/box2d-smoke, export run_shapes) in Node.
// Each check returns its own code on failure: 10s rounded polygon, 20s chain loop, 30s one-sided chain,
// 40s properties, 50s rolling resistance, 60s tangent speed. Build first:
//   cd crates/box2d-smoke && cargo build --target wasm32-unknown-unknown --release
import { readFileSync } from "node:fs";

const wasmPath = new URL("../crates/box2d-smoke/target/wasm32-unknown-unknown/release/box2d_smoke.wasm", import.meta.url);
const module = new WebAssembly.Module(readFileSync(wasmPath));
if (WebAssembly.Module.imports(module).length) throw new Error("the module should import nothing");
const code = new WebAssembly.Instance(module, {}).exports.run_shapes();
if (code !== 0) throw new Error(`shapes check failed with code ${code}`);
console.log("ok shapes: rounded polygon, chain loop and one-sided chain, property get/set, rolling resistance, tangent speed");
