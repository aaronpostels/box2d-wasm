// Queries glue through the Emscripten module: shape cast, overlaps (AABB, circle, polygon), all-hits ray cast, explosion.
// Calls the raw _b2Glue_* exports, since the TypeScript wrapper has no query methods. Same scene and expected values
// as the Rust smoke section (crates/box2d-smoke/src/lib.rs), so both builds must agree.
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";

// The Rust smoke module runs the same scene without a dist build. The dist part below is skipped locally when dist/ is
// missing, and fails when CI or REQUIRE_DIST=1 is set.
// Build the smoke wasm first: cd crates/box2d-smoke && cargo build --target wasm32-unknown-unknown --release
const smokeWasm = new URL("../crates/box2d-smoke/target/wasm32-unknown-unknown/release/box2d_smoke.wasm", import.meta.url);
const smoke = new WebAssembly.Instance(new WebAssembly.Module(readFileSync(smokeWasm)), {}).exports;
const smokeCode = smoke.run_queries();
assert.equal(smokeCode, 0, `queries smoke check failed with code ${smokeCode}`);
console.log("ok queries smoke: overlaps, ray hits nearest first, shape cast, explosion");

if (!existsSync(fileURLToPath(new URL("../dist/box2d.js", import.meta.url)))) {
  if (process.env.CI || process.env.REQUIRE_DIST === "1") {
    throw new Error("queries: no dist/box2d.js build, and CI or REQUIRE_DIST=1 requires it");
  }
  console.log("queries: no dist/box2d.js build found. Skipping the dist checks.");
  process.exit(0);
}
const { initBox2D } = await import("../dist/index.js");

const wasm = await initBox2D();
const HIT_BYTES = 32; // b2Glue_Hit: u64 shape, f32 fraction, f32 point[2], f32 normal[2]
const BODY_STATIC = 0;
const BODY_DYNAMIC = 2;

const alloc = (bytes) => wasm._malloc(bytes);
const writeF32 = (values) => {
  const p = alloc(values.length * 4);
  for (let i = 0; i < values.length; i++) wasm.HEAPF32[(p >> 2) + i] = values[i];
  return p;
};
const readF32 = (p, n) => Array.from({ length: n }, (_, i) => wasm.HEAPF32[(p >> 2) + i]);
const readIds = (p, n) => Array.from({ length: n }, (_, i) => wasm.HEAPU64[(p >> 3) + i]);
const readHit = (p) => {
  const dv = new DataView(wasm.HEAPU8.buffer, p, HIT_BYTES);
  return {
    shape: dv.getBigUint64(0, true),
    fraction: dv.getFloat32(8, true),
    point: [dv.getFloat32(12, true), dv.getFloat32(16, true)],
    normal: [dv.getFloat32(20, true), dv.getFloat32(24, true)]
  };
};
const near = (actual, expected, tol, msg) => assert.ok(Math.abs(actual - expected) <= tol, `${msg}: ${actual} vs ${expected}`);

// Scene: static box A at origin, dynamic circle B at x=5, static box C at x=10, static box D at y=10. No gravity.
const w = wasm._b2Glue_CreateWorld(0, 0, 0, 0);
const zero = writeF32([0, 0, 0]);
const makeBody = (world, x, y, kind) => wasm._b2Glue_CreateBodyEx(world, kind, writeF32([x, y, 0]), zero, 0);
const a = wasm._b2Glue_AddBox(makeBody(w, 0, 0, BODY_STATIC), 1, 1, 1, 0.5, 0, 0);
const bBody = makeBody(w, 5, 0, BODY_DYNAMIC);
const b = wasm._b2Glue_AddCircle(bBody, 0.5, 1, 0.5, 0, 0);
const c = wasm._b2Glue_AddBox(makeBody(w, 10, 0, BODY_STATIC), 1, 1, 1, 0.5, 0, 0);
wasm._b2Glue_AddBox(makeBody(w, 0, 10, BODY_STATIC), 1, 1, 1, 0.5, 0, 0);

const ids = alloc(8 * 8);

// Overlap AABB: a small box hits only A; a wide strip hits A and B, not C.
assert.equal(wasm._b2Glue_OverlapAABB(w, -2, -2, 2, 2, ids, 8), 1);
assert.deepEqual(readIds(ids, 1), [a]);
assert.equal(wasm._b2Glue_OverlapAABB(w, -6, -1, 6, 1, ids, 8), 2);
assert.ok(readIds(ids, 2).includes(a) && readIds(ids, 2).includes(b), "strip must hit A and B");
// The total is reported even when the buffer is too small.
assert.equal(wasm._b2Glue_OverlapAABB(w, -6, -1, 6, 1, ids, 1), 2);
console.log("ok queries: overlap AABB");

// Overlap circle: inside B only, then clear of everything.
assert.equal(wasm._b2Glue_OverlapCircle(w, 5, 0, 0.1, ids, 8), 1);
assert.deepEqual(readIds(ids, 1), [b]);
assert.equal(wasm._b2Glue_OverlapCircle(w, 0, 3, 0.5, ids, 8), 0);
console.log("ok queries: overlap circle");

// Overlap polygon: a triangle around B's centre, and the point count limits.
const tri = writeF32([4, -1, 6, -1, 5, 1]);
assert.equal(wasm._b2Glue_OverlapPolygon(w, tri, 3, 0, ids, 8), 1);
assert.deepEqual(readIds(ids, 1), [b]);
assert.equal(wasm._b2Glue_OverlapPolygon(w, tri, 0, 0, ids, 8), -1);
assert.equal(wasm._b2Glue_OverlapPolygon(w, writeF32(new Array(18).fill(0)), 9, 0, ids, 8), -1);
console.log("ok queries: overlap polygon");

// Ray, all hits: from x=-5 towards +x it crosses A (t=0.2), B (t=0.475) and C (t=0.7).
const hits = alloc(4 * HIT_BYTES);
const rayOrigin = writeF32([-5, 0]);
const rayTranslation = writeF32([20, 0]);
assert.equal(wasm._b2Glue_CastRayAll(w, rayOrigin, rayTranslation, hits, 4), 3);
// The glue sorts hits nearest first, so the order is checked as returned, not sorted here.
const found = [0, 1, 2].map((i) => readHit(hits + i * HIT_BYTES));
assert.equal(found[0].shape, a);
near(found[0].fraction, 0.2, 1e-4, "ray hit A fraction");
near(found[0].normal[0], -1, 1e-4, "ray hit A normal");
assert.equal(found[1].shape, b);
near(found[1].fraction, 0.475, 1e-3, "ray hit B fraction");
assert.equal(found[2].shape, c);
near(found[2].fraction, 0.7, 1e-3, "ray hit C fraction");
// A buffer shorter than the hit count still returns the full count, and keeps the two nearest hits in order.
assert.equal(wasm._b2Glue_CastRayAll(w, rayOrigin, rayTranslation, hits, 2), 3);
assert.equal(readHit(hits).shape, a, "short buffer keeps A first");
assert.equal(readHit(hits + HIT_BYTES).shape, b, "short buffer keeps B second");
console.log("ok queries: ray cast all hits");

// Shape cast: a radius 0.5 circle moved from x=-5 meets A's left face at t=0.175. Passing at y=5 misses.
const circle = writeF32([0, 0]);
const castHit = alloc(HIT_BYTES);
assert.equal(wasm._b2Glue_CastShapeClosest(w, circle, 1, 0.5, rayOrigin, rayTranslation, castHit), 1);
const h = readHit(castHit);
assert.equal(h.shape, a);
near(h.fraction, 0.175, 1e-3, "shape cast fraction");
near(h.normal[0], -1, 1e-3, "shape cast normal");
assert.equal(wasm._b2Glue_CastShapeClosest(w, circle, 1, 0.5, writeF32([-5, 5]), rayTranslation, castHit), 0);
assert.equal(wasm._b2Glue_CastShapeClosest(w, circle, 0, 0.5, rayOrigin, rayTranslation, castHit), -1);
console.log("ok queries: shape cast");
wasm._b2Glue_DestroyWorld(w);

// Explosion at the origin, radius 5, falloff 2. Surface distances 1.5 (full impulse), 5.5 (scale 0.75) and 7.5
// (outside). Equal masses, so velocity ratios equal impulse ratios.
const w2 = wasm._b2Glue_CreateWorld(0, 0, 0, 0);
const circleAt = (x) => {
  const body = makeBody(w2, x, 0, BODY_DYNAMIC);
  wasm._b2Glue_AddCircle(body, 0.5, 1, 0.5, 0, 0);
  return body;
};
const nearBody = circleAt(2);
const midBody = circleAt(6);
const farBody = circleAt(8);
wasm._b2Glue_Explode(w2, 0, 0, 5, 2, 1, 0xffffffffffffffffn);
const velocity = (body) => {
  const p = alloc(8);
  wasm._b2Glue_GetLinearVelocity(body, p);
  return readF32(p, 2);
};
const vn = velocity(nearBody);
const vm = velocity(midBody);
const vf = velocity(farBody);
assert.ok(vn[0] > 0.1 && Math.abs(vn[1]) < 1e-5, `inside radius pushed outwards: ${vn}`);
near(vm[0] / vn[0], 0.75, 1e-3, "falloff band ratio");
assert.ok(Math.abs(vm[1]) < 1e-5, `falloff band pushed sideways: ${vm}`);
assert.deepEqual(vf, [0, 0], "beyond radius + falloff must be untouched");
wasm._b2Glue_DestroyWorld(w2);
console.log("ok queries: explosion falloff");
