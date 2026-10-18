// Threaded flavour: dist/box2d.threads.js (Emscripten pthreads, -pthread, SHARED_MEMORY). Must start on shared memory, step the
// smoke scene, and agree bit for bit with the SIMD build (dist/box2d.js) when that build is present. Skips without dist/ locally,
// and fails there when CI or REQUIRE_DIST=1 is set.
import assert from "node:assert/strict";
import { pathToFileURL } from "node:url";
import { resolve } from "node:path";
import { existsSync } from "node:fs";

// Missing dist files fail when CI or REQUIRE_DIST=1 is set, and skip locally.
const required = Boolean(process.env.CI) || process.env.REQUIRE_DIST === "1";
if (!existsSync(resolve("dist", "box2d.threads.js"))) {
  if (required) throw new Error("threads: no dist/box2d.threads.js build, and CI or REQUIRE_DIST=1 requires it");
  console.log("No dist/box2d.threads.js build found. Skipping threads test (CI builds it in the Emscripten image).");
  process.exit(0);
}

// Imported after the check: dist/index.js only exists once the TypeScript build has run.
const { initBox2D, Box2DWorld, BodyType } = await import("../dist/index.js");

async function runScene(file) {
  const factory = (await import(pathToFileURL(resolve("dist", file)).href)).default;
  const wasm = await initBox2D(factory);
  const world = new Box2DWorld(wasm, [0, -10]);
  const ground = world.createBody({ type: BodyType.Static, position: [0, -1] });
  world.createBoxShape(ground, { halfWidth: 50, halfHeight: 1, friction: 0.5 });
  const ids = [];
  for (let i = 0; i < 200; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: [(i % 10) * 2.2 - 10, 2 + Math.floor(i / 10) * 2.2] });
    world.createBoxShape(b, { halfWidth: 0.9, halfHeight: 0.9, density: 1, friction: 0.5 });
    ids.push(b);
  }
  for (let i = 0; i < 240; i++) world.step(1 / 60, 4);
  const out = new Float32Array(ids.length * 3);
  world.syncBatchTransforms(ids, out);
  const result = { shared: wasm.HEAPU8.buffer instanceof SharedArrayBuffer, hash: world.getStateHash(), out };
  world.destroy();
  return result;
}

const threaded = await runScene("box2d.threads.js");
assert.ok(threaded.shared, "threads build must run on a SharedArrayBuffer heap (pthreads)");
assert.ok(threaded.out.every(Number.isFinite), "threads: non-finite transform");
assert.ok(threaded.out[1] > -0.5 && threaded.out[1] < 50, `threads: box not resting on ground (y=${threaded.out[1]})`);
console.log("ok threads: shared heap, stepped");

if (existsSync(resolve("dist", "box2d.js"))) {
  const simd = await runScene("box2d.js");
  assert.deepEqual(Array.from(threaded.out), Array.from(simd.out), "threads and SIMD builds diverge");
  assert.equal(threaded.hash, simd.hash, "threads and SIMD state hash diverge");
  console.log("ok threads == simd");
} else {
  if (required) throw new Error("threads: no dist/box2d.js build, and CI or REQUIRE_DIST=1 requires the SIMD comparison");
  console.log("No dist/box2d.js build found. Skipped the comparison with the SIMD build.");
}
