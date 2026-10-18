// Type checks for the TypeScript wrapper. Always runs: needs only the TypeScript compiler (npm install), no Emscripten build.
//   1. Drift: every B2_GLUE_EXPORT in the new glue headers (shapes, joints, queries, bodies, filters) has a matching
//      _b2Glue_ entry in RawBox2DModule (src/ts/types.ts), with the same parameter count and bigint/number kinds.
//   2. Usage: tsc --noEmit over a file that calls every wrapper method with the declared types. A wrapper signature that
//      drifts from its raw call (or loosens a bigint id to number) fails the compile.
import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { mkdirSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { createRequire } from "node:module";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const root = fileURLToPath(new URL("..", import.meta.url));
const read = (rel) => readFileSync(join(root, rel), "utf8");

// ---- 1. Header drift ----

const headers = ["shapes", "joints", "queries", "bodies", "filters"].map((n) => `src/c/box2d_glue_${n}.h`);
const decl = /^B2_GLUE_EXPORT\s+(.+?)\s+(b2Glue_\w+)\((.*)\);/gm;

// Expected raw kind of one C parameter or return: "bigint" for a plain uint64_t, "number" for everything else.
const kindOfC = (ctype) => (!ctype.includes("*") && /\buint64_t\b/.test(ctype) ? "bigint" : "number");

const rawTypes = read("src/ts/types.ts");
const rawDecl = /^\s+_(b2Glue_\w+)\((.*)\):\s*(.+);$/gm;
const raw = new Map();
for (const m of rawTypes.matchAll(rawDecl)) {
  const params = m[2].trim() === "" ? [] : m[2].split(",").map((p) => p.trim().split(":").slice(1).join(":").trim());
  raw.set(m[1], { params, ret: m[3].trim() });
}

let checked = 0;
const problems = [];
for (const rel of headers) {
  for (const m of read(rel).matchAll(decl)) {
    const [, retC, name, paramList] = m;
    const entry = raw.get(name);
    if (!entry) {
      problems.push(`${name} (${rel}) has no _${name} entry in RawBox2DModule`);
      continue;
    }
    const cParams = paramList.trim() === "" || paramList.trim() === "void" ? [] : paramList.split(",").map((p) => p.trim());
    const cKinds = cParams.map((p) => kindOfC(p.replace(/\b[A-Za-z_]\w*$/, "")));
    if (cKinds.length !== entry.params.length) {
      problems.push(`${name}: C has ${cKinds.length} params, RawBox2DModule has ${entry.params.length}`);
      continue;
    }
    cKinds.forEach((kind, i) => {
      const ts = entry.params[i];
      const ok = kind === "bigint" ? ts === "bigint" : ts.includes("number") && !ts.includes("bigint");
      if (!ok) problems.push(`${name}: parameter ${i + 1} is ${kind} in C but "${ts}" in RawBox2DModule`);
    });
    const retKind = retC.trim() === "void" ? "void" : kindOfC(retC);
    if (entry.ret !== retKind) problems.push(`${name}: return is ${retKind} in C but "${entry.ret}" in RawBox2DModule`);
    checked++;
  }
}
assert.ok(checked > 40, `header parser found only ${checked} declarations`);
assert.deepEqual(problems, [], `raw typings drifted from the glue headers:\n  ${problems.join("\n  ")}`);
console.log(`ok ts types: ${checked} glue declarations match RawBox2DModule`);

// ---- 2. Usage file through tsc ----

const require = createRequire(join(root, "package.json"));
let tscJs;
try {
  tscJs = join(dirname(require.resolve("typescript")), "..", "bin", "tsc");
} catch {
  throw new Error("typescript is not installed. Run npm install (the type test needs the TypeScript compiler).");
}

const cacheDir = join(root, "node_modules", ".cache", "box2d-ts-types");
mkdirSync(cacheDir, { recursive: true });
const usage = `
import {
  Box2DWorld, BodyType, JointType,
  type RawBox2DModule, type BodyId, type ShapeId, type JointId, type ChainId,
  type ShapeFilter, type MassData, type QueryHit, type OverlapResult, type RayAllResult, type JointReaction,
  type JointTypeValue, type Vec2Tuple
} from "../../../src/ts/index.js";

declare const wasm: RawBox2DModule;
const w = new Box2DWorld(wasm, [0, -10]);
const body: BodyId = w.createBody({ type: BodyType.Dynamic });
const other: BodyId = w.createBody({ type: BodyType.Static });
const shape: ShapeId = w.createBoxShape(body, { halfWidth: 1, halfHeight: 1 });
const v: Vec2Tuple = [0, 0];

// Shapes
const radius: number = w.getShapeRadius(shape);
const friction: number = w.getShapeFriction(shape);
w.setShapeFriction(shape, friction);
const restitution: number = w.getShapeRestitution(shape);
w.setShapeRestitution(shape, restitution);
const density: number = w.getShapeDensity(shape);
w.setShapeDensity(shape, density);
w.setShapeDensity(shape, density, false);
const sensor: boolean = w.isShapeSensor(shape);
const rolling: number = w.getShapeRollingResistance(shape);
w.setShapeRollingResistance(shape, rolling);
const tangent: number = w.getShapeTangentSpeed(shape);
w.setShapeTangentSpeed(shape, tangent);
const chain: ChainId = w.createChainShape(other, { points: [v, [1, 0], [1, 1]], isLoop: true, friction: 0.6, restitution: 0, rollingResistance: 0, tangentSpeed: 0 });
const chainOk: boolean = w.isChainValid(chain);
const segments: number = w.getChainSegmentCount(chain);
const filter: ShapeFilter = { categoryBits: 2n, maskBits: 0xffffffffffffffffn, groupIndex: -1 };
w.setShapeFilter(shape, filter);
const readFilter: ShapeFilter = w.getShapeFilter(shape);

// Bodies
const damping: number = w.getLinearDamping(body);
w.setLinearDamping(body, damping);
w.setAngularDamping(body, w.getAngularDamping(body));
w.setGravityScale(body, w.getGravityScale(body));
const fixed: boolean = w.isFixedRotation(body);
w.setFixedRotation(body, fixed);
const mass: MassData = w.getMassData(body);
w.setMassData(body, mass);
const userData: number = w.getBodyUserData(body);
w.setBodyUserData(body, userData);
w.enableBody(body);
w.disableBody(body);
const enabled: boolean = w.isBodyEnabled(body);
w.applyLinearImpulseBatch([body, other], [1, 0, 0, 1]);
w.applyLinearImpulseBatch([body], [1, 0], false);
w.applyAngularImpulseBatch([body, other], [0.5, -0.5]);

// Joints
const wheel: JointId = w.createWheelJoint({ bodyA: other, bodyB: body, axis: v, spring: { hertz: 2, dampingRatio: 0.7 }, limits: [-0.5, 0.5], motor: { speed: 1, maxTorque: 10 } });
const mouse: JointId = w.createMouseJoint({ bodyA: other, bodyB: body, target: v, anchorB: v, maxForce: 1000, hertz: 4, dampingRatio: 0.7 });
const filterJoint: JointId = w.createFilterJoint({ bodyA: other, bodyB: body });
const moved: boolean = w.setMouseJointTarget(mouse, [1, 1]);
const kind: JointTypeValue = w.getJointType(wheel);
const isWheel: boolean = kind === JointType.Wheel;
const limitsOk: boolean = w.setJointLimits(wheel, [-1, 1]);
w.enableJointLimit(wheel, true);
w.enableJointSpring(wheel, false);
w.setJointSpring(wheel, 2, 0.7);
w.enableJointMotor(wheel, true);
const motorOk: boolean = w.setJointMotor(wheel, 2, 100);
const reaction: JointReaction | null = w.getJointReaction(wheel);
void [radius, sensor, segments, chainOk, readFilter, enabled, isWheel, limitsOk, motorOk, moved, reaction, filterJoint, userData];

// Queries
const cast: QueryHit | null = w.castShapeClosest([v], 0.5, [-5, 0], [20, 0]);
const aabb: OverlapResult = w.overlapAABB([-2, -2], [2, 2]);
const aabbCapped: OverlapResult = w.overlapAABB([-2, -2], [2, 2], 1);
const circle: OverlapResult = w.overlapCircle([0, 0], 1, 8);
const polygon: OverlapResult = w.overlapPolygon([v, [1, 0], [0, 1]], 0);
const rays: RayAllResult = w.castRayAll([-5, 0], [20, 0], 4);
w.explode([0, 0], 5, 2, 1);
w.explode([0, 0], 5, 2, 1, 1n);
void [cast, aabb, aabbCapped, circle, polygon, rays];

// Wrong id types must not compile: a plain number is not a BodyId.
// @ts-expect-error BodyId is bigint
w.setLinearDamping(1, 0.5);
// @ts-expect-error ShapeId is bigint
w.getShapeFriction(1);
// @ts-expect-error the category bits are bigint
w.setShapeFilter(shape, { categoryBits: 1, maskBits: 1n, groupIndex: 0 });

w.destroy();
`;
const usagePath = join(cacheDir, "usage.ts");
writeFileSync(usagePath, usage);
const tsconfigPath = join(cacheDir, "tsconfig.json");
writeFileSync(
  tsconfigPath,
  JSON.stringify({
    compilerOptions: {
      target: "ES2022",
      module: "ESNext",
      moduleResolution: "Bundler",
      strict: true,
      noEmit: true,
      skipLibCheck: true,
      types: [],
    },
    files: ["usage.ts"],
  })
);

try {
  execFileSync(process.execPath, [tscJs, "-p", tsconfigPath], { stdio: "pipe", encoding: "utf8" });
} catch (err) {
  throw new Error(`usage file does not type-check against the wrapper:\n${err.stdout || err.stderr}`);
} finally {
  rmSync(cacheDir, { recursive: true, force: true });
}
console.log("ok ts types: usage file type-checks against the wrapper");
