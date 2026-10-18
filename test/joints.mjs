// Joints checks. First the Rust smoke module (run_joints in crates/box2d-smoke/src/lib.rs), which needs no dist build.
// Then the joints glue through the raw Emscripten module (dist/box2d.js), mirroring the same checks. Without dist/box2d.js
// the dist part is skipped locally, and fails when CI or REQUIRE_DIST=1 is set (npm run build:wasm needs Emscripten).
// Build the smoke wasm first: cd crates/box2d-smoke && cargo build --target wasm32-unknown-unknown --release
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";

const smokeWasm = new URL("../crates/box2d-smoke/target/wasm32-unknown-unknown/release/box2d_smoke.wasm", import.meta.url);
const smoke = new WebAssembly.Instance(new WebAssembly.Module(readFileSync(smokeWasm)), {}).exports;
const smokeCode = smoke.run_joints();
assert.equal(smokeCode, 0, `joints smoke check failed with code ${smokeCode}`);
console.log("ok joints smoke: kinds, revolute limit and motor, prismatic, distance, wheel, mouse, filter, reaction");

const entry = new URL("../dist/box2d.js", import.meta.url);
if (!existsSync(fileURLToPath(entry))) {
  if (process.env.CI || process.env.REQUIRE_DIST === "1") {
    throw new Error("joints: no dist/box2d.js build, and CI or REQUIRE_DIST=1 requires it");
  }
  console.log("joints: no dist/box2d.js build found. Skipping the dist checks.");
  process.exit(0);
}
const m = await (await import(entry.href)).default();

const BODY_STATIC = 0;
const BODY_DYNAMIC = 2;
const JOINT_FILTER = 1;
const JOINT_MOTOR = 2;
const JOINT_WHEEL = 8;

const f32 = values => {
  const p = m._malloc(values.length * 4);
  m.HEAPF32.set(values, p >> 2);
  return p;
};
const readF32 = (p, n) => Array.from(m.HEAPF32.subarray(p >> 2, (p >> 2) + n));

function world(gravityY) {
  return m._b2Glue_CreateWorld(0, gravityY, 0, 0);
}

function body(w, type, x, y) {
  const pose = f32([x, y, 0]);
  const vel = f32([0, 0, 0]);
  const b = m._b2Glue_CreateBodyEx(w, type, pose, vel, 0);
  m._free(pose);
  m._free(vel);
  return b;
}

function box(w, type, x, y, half) {
  const b = body(w, type, x, y);
  m._b2Glue_AddBox(b, half, half, 1, 0.5, 0, 0);
  return b;
}

function steps(w, n) {
  for (let i = 0; i < n; i++) m._b2Glue_Step(w, 1 / 60, 4);
}

function pose(b) {
  const p = m._malloc(12);
  m._b2Glue_GetTransform(b, p);
  const out = readF32(p, 3);
  m._free(p);
  return out;
}

function vec2(x, y) {
  return f32([x, y]);
}

// Wheel limit: a slider kicked along its vertical axis stops at +-0.5.
{
  const w = world(0);
  const base = body(w, BODY_STATIC, 0, 0);
  const slider = box(w, BODY_DYNAMIC, 0, 0, 0.5);
  const axis = vec2(0, 1);
  const anchor = vec2(0, 0);
  const wheel = m._b2Glue_CreateWheel(w, base, slider, anchor, anchor, axis, 0, 0, 0, 1, -0.5, 0.5, 0, 0, 0);
  assert.equal(m._b2Glue_JointType(wheel), JOINT_WHEEL, "wheel kind");
  m._b2Glue_ApplyLinearImpulseToCenter(slider, 0, 5, 1);
  steps(w, 120);
  const y = pose(slider)[1];
  assert.ok(y >= 0.45 && y <= 0.52, `wheel limit held at 0.5, got ${y}`);
  m._b2Glue_DestroyWorld(w);
}

// Distance spring: a rigid rod holds length 2; a 2 Hz spring lets the weight sag by g / w^2 = 0.063 m.
{
  const w = world(-10);
  const ceiling = body(w, BODY_STATIC, 0, 10);
  const weight = box(w, BODY_DYNAMIC, 0, 8, 0.5);
  const anchor = vec2(0, 0);
  const d = m._b2Glue_CreateDistance(w, ceiling, weight, anchor, anchor, 2, 0, 0);
  steps(w, 240);
  assert.ok(Math.abs(pose(weight)[1] - 8) < 0.05, "rigid rod holds length 2");
  assert.equal(m._b2Glue_JointSetSpring(d, 2, 0.7), 1, "distance has a spring");
  assert.equal(m._b2Glue_JointEnableSpring(d, 1), 1, "enable spring");
  steps(w, 480);
  const y = pose(weight)[1];
  assert.ok(y > 7.85 && y < 7.99, `spring sag, got ${y}`);
  m._b2Glue_DestroyWorld(w);
}

// Mouse: the pulled point follows the target, the target can move, and the setter refuses other kinds.
{
  const w = world(0);
  const ground = body(w, BODY_STATIC, 0, 0);
  const b = box(w, BODY_DYNAMIC, 0, 0, 0.5);
  const target = vec2(2, 0);
  const anchor = vec2(0, 0);
  const mouse = m._b2Glue_CreateMouse(w, ground, b, target, anchor, 1000, 4, 0.7);
  assert.equal(m._b2Glue_JointType(mouse), JOINT_MOTOR, "mouse is a motor joint");
  steps(w, 180);
  assert.ok(Math.abs(pose(b)[0] - 2) < 0.2, "mouse pulls to target");
  assert.equal(m._b2Glue_MouseSetTarget(mouse, -2, 0), 1, "mouse target moves");
  steps(w, 180);
  assert.ok(Math.abs(pose(b)[0] + 2) < 0.2, "mouse follows the moved target");
  const filter = m._b2Glue_CreateFilter(w, ground, b);
  assert.equal(m._b2Glue_MouseSetTarget(filter, 0, 0), 0, "setter refuses a filter joint");
  // A destroyed joint id is stale: the glue returns its failure value instead of asserting inside Box2D.
  m._b2Glue_DestroyJoint(filter);
  assert.equal(m._b2Glue_JointType(filter), -1, "stale id has no kind");
  assert.equal(m._b2Glue_JointSetLimits(filter, 0, 1), 0, "stale id refuses a limit");
  assert.equal(m._b2Glue_MouseSetTarget(filter, 0, 0), 0, "stale id refuses a mouse target");
  m._b2Glue_DestroyWorld(w);
}

// Filter: a box joined to the floor by a filter joint falls through it; without the joint it rests on it.
{
  const w = world(-10);
  const floor = body(w, BODY_STATIC, 0, 0);
  m._b2Glue_AddBox(floor, 10, 0.5, 1, 0.5, 0, 0);
  const falling = box(w, BODY_DYNAMIC, 0, 1, 0.5);
  const filter = m._b2Glue_CreateFilter(w, floor, falling);
  assert.equal(m._b2Glue_JointType(filter), JOINT_FILTER, "filter kind");
  assert.equal(m._b2Glue_JointSetLimits(filter, 0, 1), 0, "filter has no limit");
  steps(w, 120);
  assert.ok(pose(falling)[1] < -1, "filter joint lets the box fall through");
  m._b2Glue_DestroyWorld(w);

  const w2 = world(-10);
  const floor2 = body(w2, BODY_STATIC, 0, 0);
  m._b2Glue_AddBox(floor2, 10, 0.5, 1, 0.5, 0, 0);
  const resting = box(w2, BODY_DYNAMIC, 0, 1, 0.5);
  steps(w2, 120);
  assert.ok(Math.abs(pose(resting)[1] - 1) < 0.05, "without the filter the box rests on the floor");
  m._b2Glue_DestroyWorld(w2);
}

// Reaction: a box hanging from a rigid rod (mass 1, g 10) is held by a 10 N pull with no torque.
{
  const w = world(-10);
  const ceiling = body(w, BODY_STATIC, 0, 2);
  const hanging = box(w, BODY_DYNAMIC, 0, 0, 0.5);
  const anchor = vec2(0, 0);
  const rod = m._b2Glue_CreateDistance(w, ceiling, hanging, anchor, anchor, 2, 0, 0);
  steps(w, 240);
  const out = m._malloc(12);
  assert.equal(m._b2Glue_JointReaction(rod, out), 1, "reaction of a valid joint");
  const [fx, fy, torque] = readF32(out, 3);
  m._free(out);
  assert.ok(Math.abs(Math.hypot(fx, fy) - 10) < 0.5, `reaction magnitude ~10, got ${Math.hypot(fx, fy)}`);
  assert.ok(Math.abs(torque) < 0.5, `reaction torque ~0, got ${torque}`);
  m._b2Glue_DestroyWorld(w);
}

console.log("ok joints: wheel limit, distance spring, mouse, filter, reaction");
