// Snapshot test: world snapshots restore bit for bit.
import assert from "node:assert/strict";
import { pathToFileURL } from "node:url";
import { resolve } from "node:path";
import { existsSync } from "node:fs";
import { initBox2D, Box2DWorld, BodyType } from "../dist/index.js";

const file = "box2d.js";
if (!existsSync(resolve("dist", file))) {
  console.log("No dist/box2d.js build found. Skipping snapshot test.");
  process.exit(0);
}

const factory = (await import(pathToFileURL(resolve("dist", file)).href)).default;
const wasm = await initBox2D(factory);
const world = new Box2DWorld(wasm, [0, -10]);

const ground = world.createBody({ type: BodyType.Static, position: [0, -1] });
world.createBoxShape(ground, { halfWidth: 50, halfHeight: 1, friction: 0.5 });
const ids = [];
for (let i = 0; i < 50; i++) {
  const b = world.createBody({ type: BodyType.Dynamic, position: [(i % 5) * 2 - 5, 2 + Math.floor(i / 5) * 2] });
  world.createBoxShape(b, { halfWidth: 0.8, halfHeight: 0.8, density: 1, friction: 0.5 });
  ids.push(b);
}

for (let i = 0; i < 60; i++) world.step(1 / 60, 4);

// Snapshot
const snapSize = wasm._b2Glue_SnapshotSize(world.id);
assert.ok(snapSize > 0, "snapshot size must be > 0");
const bufPtr = wasm._malloc(snapSize);
const written = wasm._b2Glue_SnapshotTake(world.id, bufPtr, snapSize);
assert.equal(written, snapSize);
const snapshotData = new Uint8Array(wasm.HEAPU8.buffer, bufPtr, snapSize).slice();

for (let i = 0; i < 60; i++) world.step(1 / 60, 4);
const stateAfterStep = new Float32Array(ids.length * 3);
world.syncBatchTransforms(ids, stateAfterStep);

// Restore
const restoreBufPtr = wasm._malloc(snapshotData.length);
new Uint8Array(wasm.HEAPU8.buffer, restoreBufPtr, snapshotData.length).set(snapshotData);
const ok = wasm._b2Glue_SnapshotRestore(world.id, restoreBufPtr, snapshotData.length);
assert.ok(ok, "b2Glue_SnapshotRestore failed");

for (let i = 0; i < 60; i++) world.step(1 / 60, 4);
const stateAfterRestore = new Float32Array(ids.length * 3);
world.syncBatchTransforms(ids, stateAfterRestore);

assert.deepEqual(Array.from(stateAfterStep), Array.from(stateAfterRestore), "restored world diverged");
console.log("ok snapshot: world restored bit for bit");

wasm._free(bufPtr);
wasm._free(restoreBufPtr);
world.destroy();
