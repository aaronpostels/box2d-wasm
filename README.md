# box2d-wasm

[Box2D](https://github.com/erincatto/box2d) (Erin Catto's modern C 2D physics engine, v3) compiled to WebAssembly, rebuilt automatically whenever upstream changes.

Two ways to use it:

| | For | What you get |
|---|---|---|
| **JS / TS** | web apps, any JS engine | `dist/box2d.wasm` (SIMD), a scalar build, a threaded build, and a TypeScript wrapper |
| **Rust** | Rust engines that target the browser | crate `box2d-sys`: Box2D linked into your own `wasm32-unknown-unknown` module, no JS in the loop, zero imports |

Box2D tracks its `main` branch: a daily GitHub Actions job pulls upstream, builds, runs the tests, commits the new submodule revision and publishes a release.

## Status

| Area | State |
|---|---|
| Bodies (static, kinematic, dynamic), circle / box / capsule / polygon / segment shapes | done |
| Weld, revolute, prismatic, distance, motor joints | done |
| Contact and sensor events, ray cast, character mover (capsule slide) | done |
| Batch read of body state (one call for all bodies) | done |
| Exact world snapshot and in-place restore (continues bit for bit) | done |
| Deterministic state hash (`b2World_GetStateHash`) | done |
| SIMD and scalar builds | tested |
| Worker threads | Rust crate on a Rayon pool (`use_rayon`); threaded WASM build with Emscripten pthreads |

## JS / TS

```bash
npm i github:aaronpostels/box2d-wasm
```

```ts
import { initBox2D, Box2DWorld, BodyType } from "box2d-wasm";

const wasm = await initBox2D();
const world = new Box2DWorld(wasm, [0, -9.81], { sleep: true, continuous: true });

const ground = world.createBody({ type: BodyType.Static, position: [0, -1] });
world.createBoxShape(ground, { halfWidth: 50, halfHeight: 1 });

const ball = world.createBody({ type: BodyType.Dynamic, position: [0, 10] });
world.createCircleShape(ball, { radius: 0.5, density: 1, restitution: 0.6 });

const ids = [ground, ball];
const out = new Float32Array(ids.length * 3); // px py rot per body
world.step(1 / 60, 4);
world.syncBatchTransforms(ids, out);          // no allocation
```

## Rust

```toml
box2d-sys = { git = "https://github.com/aaronpostels/box2d-wasm", rev = "<commit>" }
```

Needs `clang` with the wasm32 target (`CC_wasm32_unknown_unknown=clang`, `AR_wasm32_unknown_unknown=llvm-ar`). `build.rs` compiles Box2D and the glue; a tiny libc (`crates/box2d-sys/include/libc`, `src/shim.c`) stands in for the C library, and Box2D's SSE2 code maps to WASM SIMD through Emscripten's header-only intrinsics. It also builds for the host (MSVC, clang, gcc) for native tests. If your Rust build uses shared memory (`+atomics`), the C is built with atomics too.

The API is flat: ids are integers, data moves in float buffers. See `crates/box2d-sys/src/lib.rs`; `crates/box2d-smoke` is a complete test module (step, events, ray, snapshot and restore) that CI runs in Node.

## Build locally

You do not need to install Emscripten locally, Docker is enough:

```bash
docker run --rm -v "$PWD:/src" -w /src emscripten/emsdk:4.0.18 bash -c \
  "bash scripts/build.sh simd && bash scripts/build.sh scalar && bash scripts/build.sh threads"
npm install && npm run build:ts && npm test
```

Rust: `cd crates/box2d-smoke && cargo build --target wasm32-unknown-unknown --release && node run.mjs`.

Clone with `--recurse-submodules` (Box2D is `vendor/box2d`).

## License

MIT. Third-party notices: [THIRD_PARTY.md](THIRD_PARTY.md).
