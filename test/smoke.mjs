// Smoke test: every built flavor must drop boxes on the ground and settle. Scalar and SIMD must agree bit for bit.
import assert from "node:assert/strict";
import { pathToFileURL } from "node:url";
import { resolve } from "node:path";
import { existsSync } from "node:fs";
import { initBox2D, Box2DWorld, BodyType } from "../dist/index.js";

const flavors = ["box2d.js", "box2d.scalar.js"].filter(f => existsSync(resolve("dist", f)));
if (flavors.length === 0) {
  console.log("No dist/box2d*.js builds found. Run npm run build:wasm first.");
  process.exit(0);
}

const results = [];
for (const flavor of flavors) {
  const factory = (await import(pathToFileURL(resolve("dist", flavor)).href)).default;
  const world = new Box2DWorld(await initBox2D(factory), [0, -10]);
  const ground = world.createBody({ type: BodyType.Static, position: [0, -1] });
  world.createBoxShape(ground, { halfWidth: 50, halfHeight: 1, friction: 0.5 });
  const ids = [];
  for (let i = 0; i < 200; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: [(i % 10) * 2.2 - 10, 2 + Math.floor(i / 10) * 2.2] });
    world.createBoxShape(b, { halfWidth: 0.9, halfHeight: 0.9, density: 1, friction: 0.5 });
    ids.push(b);
  }
  const out = new Float32Array(ids.length * 3);
  for (let i = 0; i < 240; i++) world.step(1 / 60, 4);
  world.syncBatchTransforms(ids, out);
  assert.ok(out.every(Number.isFinite), `${flavor}: non-finite transform`);
  assert.ok(out[1] > -0.5 && out[1] < 50, `${flavor}: box not resting on ground (y=${out[1]})`);
  results.push(Array.from(out));
  world.destroy();
  console.log(`ok ${flavor}`);
}

if (results.length > 1) {
  assert.deepEqual(results[0], results[1], "SIMD and scalar builds diverge");
  console.log("ok simd == scalar");
}
