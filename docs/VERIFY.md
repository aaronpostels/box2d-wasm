# Verification

What proves the box2d-wasm builds are right, what runs where, and what was checked by hand. Built on agent/P2e (2026-10-18).

## 1. Threaded build test (`test/threads.mjs`)

- Loads `dist/box2d.threads.js` (Emscripten `-pthread`, `SHARED_MEMORY`), checks the heap is a `SharedArrayBuffer`, steps the smoke scene (200 boxes, 240 steps) and checks the ground holds.
- When `dist/box2d.js` (SIMD) exists it compares transforms and `getStateHash()` bit for bit with the threaded build.
- Without `dist/box2d.threads.js` it prints a skip line and exits 0 (local machines without Emscripten).
- CI: step "Threaded build test" in `.github/workflows/ci.yml`, after the smoke test.
- Not run locally: Docker is broken here, so no Emscripten build. First real run is the CI job.
- Limit: the JS wrapper has no task-system call, so the threaded build steps Box2D on one thread. Parallel solving is only reachable from the Rust crate (`use_rayon`). The test proves the pthread build runs and matches, not that worker threads do the work.

## 2. Determinism across worker counts (native)

Check: the same scene stepped with 1, 2, 4 and 8 workers must give the same position hash. Hash is FNV-1a over the f32 bits of px, py and rotation of every body, plus Box2D's `b2World_GetStateHash`.

- Binary: `benches/native_physics/src/bin/det2d.rs` in the Artisan repo (wave/P2e). It uses the pinned `box2d-sys` (rev 6c3b33d), builds its scene from a fixed LCG (no files, no clock), and prints one `DET {json}` line.
- Run: `cargo run --release --bin det2d -- <workers> <steps> <bodies>` inside `benches/native_physics`.
- `effectiveWorkers` and `tasks` in the output show the pool was really used (tasks = 0 at 1 worker).

Results (600 steps, sleep and continuous on, MSVC host build, SSE2 width 4):

| bodies | workers | effective | tasks handed to pool | posHash | stateHash |
|---|---|---|---|---|---|
| 400 | 1 | 1 | 0 | 405331cf4940718d | 6c69ed0f913b45de |
| 400 | 2 | 2 | 5702 | 405331cf4940718d | 6c69ed0f913b45de |
| 400 | 4 | 4 | 9771 | 405331cf4940718d | 6c69ed0f913b45de |
| 400 | 8 | 8 | 16649 | 405331cf4940718d | 6c69ed0f913b45de |
| 2000 | 1 | 1 | 0 | 8018124af9afc918 | e291872f3dbdabd1 |
| 2000 | 2 | 2 | 5976 | 8018124af9afc918 | e291872f3dbdabd1 |
| 2000 | 4 | 4 | 10776 | 8018124af9afc918 | e291872f3dbdabd1 |
| 2000 | 8 | 8 | 20373 | 8018124af9afc918 | e291872f3dbdabd1 |

Same hashes for the scalar-solver native build (`BOX2D_SCALAR=1`) at 1 and 8 workers, both sizes. No timings were taken.

## 3. SIMD audit (is the wasm solver the SIMD one?)

How the solver is chosen, from `vendor/box2d/src/core.h`:

- `BOX2D_DISABLE_SIMD` gives `B2_SIMD_NONE` (scalar lanes). Otherwise `B2_CPU_WASM` gives `B2_SIMD_SSE2`, which the wide solver (`contact_solver_wide.inl`, width 4 in `contact_solver_w4.c`) maps to wasm SIMD128 through the SSE headers.
- `B2_CPU_WASM` comes from `__EMSCRIPTEN__` (Emscripten build) or from `-DB2_CPU_WASM` (`crates/box2d-sys/build.rs`, clang). Without either, `core.h` falls through to `B2_SIMD_NONE` with no warning. This is the same failure as the Box3D `B3_CPU_WASM` bug fixed earlier.
- `scripts/build.sh simd` and `threads` pass `-msimd128 -msse2` and no disable flag. `scripts/build.sh scalar` passes `-DBOX2D_DISABLE_SIMD` and no `-msimd128`.

Evidence, from compiling the same C sources (`vendor/box2d/src`, `src/c/box2d_glue*.c`, timer.c excluded as in build.rs) with clang `--target=wasm32-unknown-unknown -O3 -ffp-contract=off` and the same defines, then `llvm-objdump -d`:

| build | SIMD instructions in .o files | `b2SolveContacts_WideW4` | `b2PrepareContacts_WideW4` | `b2PushContacts_WideW4` |
|---|---|---|---|---|
| SIMD (`-msimd128`) | 4948 (`v128.*`, `f32x4.*`, ...) | 319 | 243 | 220 |
| scalar (`-DBOX2D_DISABLE_SIMD`) | 0 | 0 | 0 | 0 |

The scalar build still contains the wide solver symbols (its lanes are plain floats), so symbol names alone do not tell the two apart. The opcode count does.

CI guard: step "SIMD audit" in `.github/workflows/ci.yml` runs `wasm-objdump -d` (wabt) on `dist/box2d.wasm`, `dist/box2d.threads.wasm` and `dist/box2d.scalar.wasm`, and fails unless the SIMD and threads builds have SIMD ops and the scalar build has none.

Not verified here: the real Emscripten output (no emcc locally). The clang compile uses the same flags except the Emscripten libc and `__EMSCRIPTEN__`. The CI audit is the check on the real artefacts.

## Findings

- Solver selection is correct in both paths (SIMD: `-msimd128`, no disable flag; scalar: `BOX2D_DISABLE_SIMD`). Nothing silently falls back to scalar.
- Silent-fallback risk: a new build path that defines neither `__EMSCRIPTEN__` nor `B2_CPU_WASM` gets the scalar solver with no error. The CI SIMD audit catches this for the Emscripten outputs.
- The threaded build cannot run the solver in parallel from JS (no task-system export in `box2d_glue.c` reachable from the wrapper). Documented above, not fixed here (glue is out of scope for this check).
- Determinism holds for 1, 2, 4, 8 workers on native (identical position and state hashes), and for SIMD vs scalar native.
