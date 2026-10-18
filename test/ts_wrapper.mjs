// Runs every method added to the TypeScript wrapper (Box2DWorld) against the built module, with the same scenes and
// expected values as the Rust smoke checks in crates/box2d-smoke/src/lib.rs (shapes, joints, queries, bodies, filters).
// Skips cleanly unless dist/index.js and dist/box2d.js exist (npm run build needs Emscripten).
import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";

const distFile = (name) => fileURLToPath(new URL(`../dist/${name}`, import.meta.url));
if (!existsSync(distFile("index.js")) || !existsSync(distFile("box2d.js"))) {
  console.log("ts wrapper: no dist/index.js or dist/box2d.js build found. Skipping.");
  process.exit(0);
}

const { initBox2D, Box2DWorld, BodyType, JointType } = await import(new URL("../dist/index.js", import.meta.url).href);
const wasm = await initBox2D();
const ALL_BITS = 0xffffffffffffffffn;
const near = (actual, expected, tol, msg) => assert.ok(Math.abs(actual - expected) <= tol, `${msg}: ${actual} vs ${expected}`);

// Zero-sleep, zero-continuous world like the Rust smoke scenes.
const world = (gravityY = -10) => new Box2DWorld(wasm, [0, gravityY], { sleep: false, continuous: false });
const steps = (w, n) => {
  for (let i = 0; i < n; i++) w.step(1 / 60, 4);
};
const floor = (w) => {
  const ground = w.createBody({ type: BodyType.Static, position: [0, -1] });
  return w.createBoxShape(ground, { halfWidth: 50, halfHeight: 1, density: 1, friction: 0.5, restitution: 0 });
};
const dynamicBox = (w, x, y, half, density = 1) => {
  const b = w.createBody({ type: BodyType.Dynamic, position: [x, y] });
  w.createBoxShape(b, { halfWidth: half, halfHeight: half, density, friction: 0.5 });
  return b;
};
const staticBody = (w, x, y) => w.createBody({ type: BodyType.Static, position: [x, y] });
const dynamicCircle = (w, x, y, friction = 0.5, restitution = 0) => {
  const b = w.createBody({ type: BodyType.Dynamic, position: [x, y] });
  const shape = w.createCircleShape(b, { radius: 0.5, density: 1, friction, restitution });
  return { body: b, shape };
};
const pos = (w, b) => w.getTransform(b).position;
const vel = (w, b) => w.getLinearVelocity(b);

// ---- Shapes ----

{
  // Rounded polygon: a 2 x 2 square with radius 0.25 rests with its center at 1.25 (a sharp box would sit at 1.0).
  const w = world();
  floor(w);
  const body = w.createBody({ type: BodyType.Dynamic, position: [0, 4] });
  const poly = w.createPolygonShape(body, { points: [[-1, -1], [1, -1], [1, 1], [-1, 1]], radius: 0.25, density: 1, friction: 0.5 });
  near(w.getShapeRadius(poly), 0.25, 1e-6, "rounded polygon radius");
  steps(w, 240);
  const y = pos(w, body)[1];
  assert.ok(y > 1.1 && y < 1.4, `rounded polygon rests at 1.25, got ${y}`);
  w.destroy();
  console.log("ok ts shapes: rounded polygon");
}

{
  // Closed clockwise loop: a ball stays inside, resting on the bottom edge (y = -5).
  const w = world();
  const frame = staticBody(w, 0, 0);
  const chain = w.createChainShape(frame, { points: [[-5, -5], [-5, 5], [5, 5], [5, -5]], isLoop: true, friction: 0.6 });
  assert.equal(w.isChainValid(chain), true, "loop chain is valid");
  assert.equal(w.getChainSegmentCount(chain), 4, "loop has 4 segments");
  const b =w.createBody({ type: BodyType.Dynamic, position: [0, 3] });
  w.createCircleShape(b, { radius: 0.5, density: 1, friction: 0.5 });
  steps(w, 300);
  const p = pos(w, b);
  assert.ok(Math.abs(p[1] + 4.5) <= 0.1 && Math.abs(p[0]) <= 4.5, `ball inside the loop, got ${p}`);
  w.destroy();
  console.log("ok ts shapes: chain loop");
}

{
  // One-sided chain, right to left: solid on top, open underneath.
  const line = [[5, 0], [0, 0], [-5, 0]];
  const top = world();
  const frame = staticBody(top, 0, 0);
  const chain = top.createChainShape(frame, { points: line, isLoop: false, friction: 0.6 });
  assert.equal(top.getChainSegmentCount(chain), 2, "open chain of 3 points has 2 segments");
  const b = top.createBody({ type: BodyType.Dynamic, position: [2, 3] });
  top.createCircleShape(b, { radius: 0.5, density: 1, friction: 0.5 });
  steps(top, 180);
  assert.ok(Math.abs(pos(top, b)[1] - 0.5) <= 0.1, `ball lands on the top face, got ${pos(top, b)[1]}`);
  top.destroy();

  const below = world();
  const frame2 = staticBody(below, 0, 0);
  below.createChainShape(frame2, { points: line, isLoop: false, friction: 0.6 });
  const c = below.createBody({ type: BodyType.Dynamic, position: [-2, -3] });
  below.createCircleShape(c, { radius: 0.5, density: 1, friction: 0.5 });
  below.setLinearVelocity(c, [0, 8]);
  let highest = -Infinity;
  for (let i = 0; i < 60; i++) {
    below.step(1 / 60, 4);
    highest = Math.max(highest, pos(below, c)[1]);
  }
  assert.ok(highest > 0, `one-sided chain is open from below, highest ${highest}`);
  below.destroy();
  console.log("ok ts shapes: one-sided chain");
}

{
  // Properties: friction, restitution, density get and set; density change updates the body mass; sensor is read-only.
  const w = world();
  const body = w.createBody({ type: BodyType.Dynamic, position: [0, 10] });
  const shape = w.createBoxShape(body, { halfWidth: 0.5, halfHeight: 0.5, density: 1, friction: 0.5, restitution: 0.2 });
  near(w.getShapeFriction(shape), 0.5, 1e-6, "friction");
  near(w.getShapeRestitution(shape), 0.2, 1e-6, "restitution");
  near(w.getShapeDensity(shape), 1, 1e-6, "density");
  near(w.getBodyMass(body), 1, 1e-4, "unit box mass");
  assert.equal(w.isShapeSensor(shape), false, "not a sensor");
  w.setShapeFriction(shape, 0.9);
  w.setShapeRestitution(shape, 0.7);
  w.setShapeDensity(shape, 3, true);
  near(w.getShapeFriction(shape), 0.9, 1e-6, "friction set");
  near(w.getShapeRestitution(shape), 0.7, 1e-6, "restitution set");
  near(w.getShapeDensity(shape), 3, 1e-6, "density set");
  near(w.getBodyMass(body), 3, 1e-4, "mass after density change");
  const sensor = w.createBoxShape(body, { halfWidth: 0.1, halfHeight: 0.1, sensor: true });
  assert.equal(w.isShapeSensor(sensor), true, "sensor flag from creation");
  w.destroy();
  console.log("ok ts shapes: properties");
}

// Rolling resistance: a ball rolling at 5 m/s slows when resistance is on.
const rollSpeed = (rolling) => {
  const w = world();
  floor(w);
  const ball = w.createBody({ type: BodyType.Dynamic, position: [0, 0.5] });
  const shape = w.createCircleShape(ball, { radius: 0.5, density: 1, friction: 0.6 });
  w.setShapeRollingResistance(shape, rolling);
  assert.equal(w.getShapeRollingResistance(shape), rolling, "rolling resistance round trip");
  w.setLinearVelocity(ball, [5, 0]);
  w.setAngularVelocity(ball, -10);
  steps(w, 120);
  const v = vel(w, ball)[0];
  w.destroy();
  return v;
};
{
  const free = rollSpeed(0);
  const braked = rollSpeed(1);
  assert.ok(free >= 4, `free ball keeps rolling, got ${free}`);
  assert.ok(braked <= free - 0.5, `rolling resistance slows the ball: ${braked} vs ${free}`);
  console.log("ok ts shapes: rolling resistance");
}

// Tangent speed: a belt under a box carries it along x.
const beltTravel = (tangent) => {
  const w = world();
  const ground = floor(w);
  w.setShapeTangentSpeed(ground, tangent);
  assert.equal(w.getShapeTangentSpeed(ground), tangent, "tangent speed round trip");
  const b = dynamicBox(w, 0, 1, 0.5);
  steps(w, 180);
  const x = pos(w, b)[0];
  w.destroy();
  return x;
};
{
  assert.ok(Math.abs(beltTravel(0)) <= 0.1, "no belt: the box stays put");
  assert.ok(Math.abs(beltTravel(3)) >= 1.0, "belt at 3 m/s carries the box");
  console.log("ok ts shapes: tangent speed");
}

// ---- Joints ----

{
  // Revolute limit: a pendulum released at 1 rad stays inside its +-0.3 rad limit.
  const w = world();
  const pivot = staticBody(w, 0, 5);
  const bob = dynamicBox(w, 0, 4, 0.5);
  const theta = 1.0;
  const rev = w.createRevoluteJoint({ bodyA: pivot, bodyB: bob, anchorA: [0, 0], anchorB: [0, 1], limits: [-0.3, 0.3] });
  assert.equal(w.getJointType(rev), JointType.Revolute, "revolute kind");
  w.setTransform(bob, [Math.sin(theta), 5 - Math.cos(theta)], theta);
  steps(w, 60);
  for (let i = 0; i < 180; i++) {
    steps(w, 1);
    assert.ok(Math.abs(w.getTransform(bob).rotation) <= 0.33, "revolute limit holds");
  }
  w.destroy();
  console.log("ok ts joints: revolute limit");
}

{
  // Revolute motor: with no gravity a free wheel reaches 2 rad/s.
  const w = world(0);
  const hub = staticBody(w, 0, 0);
  const wheel = dynamicBox(w, 3, 0, 0.5);
  const rev = w.createRevoluteJoint({ bodyA: hub, bodyB: wheel, anchorA: [0, 0], anchorB: [-3, 0] });
  assert.equal(w.enableJointMotor(rev, true), true, "revolute has a motor");
  assert.equal(w.setJointMotor(rev, 2, 1000), true, "motor speed set");
  steps(w, 60);
  near(w.getAngularVelocity(wheel), 2, 0.05, "revolute motor speed");
  w.destroy();
  console.log("ok ts joints: revolute motor");
}

{
  // Prismatic limit: a slider kicked along x stops at +-0.5.
  const w = world(0);
  const rail = staticBody(w, 0, 0);
  const slider = dynamicBox(w, 0, 0, 0.5);
  const pr = w.createPrismaticJoint({ bodyA: rail, bodyB: slider, axis: [1, 0] });
  assert.equal(w.setJointLimits(pr, [-0.5, 0.5]), true, "prismatic has limits");
  assert.equal(w.enableJointLimit(pr, true), true, "limit enabled");
  w.applyLinearImpulseToCenter(slider, [5, 0], true);
  steps(w, 120);
  const x = pos(w, slider)[0];
  assert.ok(x >= 0.45 && x <= 0.52, `prismatic limit holds at 0.5, got ${x}`);
  w.destroy();
  console.log("ok ts joints: prismatic limit");
}

{
  // Prismatic spring: a slider hanging on a vertical axis sags by g / w^2 = 0.063 m at 2 Hz.
  const w = world();
  const rail = staticBody(w, 0, 0);
  const slider = dynamicBox(w, 0, 0, 0.5);
  const pr = w.createPrismaticJoint({ bodyA: rail, bodyB: slider, axis: [0, 1] });
  assert.equal(w.setJointSpring(pr, 2, 0.7), true, "prismatic has a spring");
  assert.equal(w.enableJointSpring(pr, true), true, "spring enabled");
  steps(w, 480);
  const y = pos(w, slider)[1];
  assert.ok(y > -0.08 && y < -0.05, `prismatic sag, got ${y}`);
  w.destroy();
  console.log("ok ts joints: prismatic spring");
}

{
  // Distance: a rigid rod holds length 2; a 2 Hz spring lets the weight sag.
  const w = world();
  const ceiling = staticBody(w, 0, 10);
  const weight = dynamicBox(w, 0, 8, 0.5);
  const d = w.createDistanceJoint({ bodyA: ceiling, bodyB: weight, length: 2, hertz: 0, dampingRatio: 0 });
  steps(w, 240);
  assert.ok(Math.abs(pos(w, weight)[1] - 8) <= 0.05, "rigid rod holds length");
  assert.equal(w.setJointSpring(d, 2, 0.7), true, "distance spring set");
  assert.equal(w.enableJointSpring(d, true), true, "distance spring on");
  steps(w, 480);
  const y = pos(w, weight)[1];
  assert.ok(y >= 7.85 && y < 7.99, `distance spring sag, got ${y}`);
  w.destroy();
  console.log("ok ts joints: distance spring");
}

{
  // Wheel limit: the wheel slides along its vertical axis and stops at +-0.5.
  const w = world(0);
  const base = staticBody(w, 0, 0);
  const slider = dynamicBox(w, 0, 0, 0.5);
  const wheel = w.createWheelJoint({ bodyA: base, bodyB: slider, axis: [0, 1], limits: [-0.5, 0.5] });
  assert.equal(w.getJointType(wheel), JointType.Wheel, "wheel kind");
  w.applyLinearImpulseToCenter(slider, [0, 5], true);
  steps(w, 120);
  const y = pos(w, slider)[1];
  assert.ok(y >= 0.45 && y <= 0.52, `wheel limit holds at 0.5, got ${y}`);
  w.destroy();
  console.log("ok ts joints: wheel limit");
}

{
  // Wheel motor: reaches 3 rad/s with no gravity. Motor set at creation.
  const w = world(0);
  const base = staticBody(w, 0, 0);
  const wheelBody = dynamicBox(w, 0, 0, 0.5);
  w.createWheelJoint({ bodyA: base, bodyB: wheelBody, axis: [1, 0], motor: { speed: 3, maxTorque: 1000 } });
  steps(w, 60);
  near(w.getAngularVelocity(wheelBody), 3, 0.1, "wheel motor speed");
  w.destroy();
  console.log("ok ts joints: wheel motor");
}

{
  // Mouse: the pulled point follows the target, the target can move, and the setter refuses non-mouse joints.
  const w = world(0);
  const ground = staticBody(w, 0, 0);
  const body = dynamicBox(w, 0, 0, 0.5);
  const mouse = w.createMouseJoint({ bodyA: ground, bodyB: body, target: [2, 0], anchorB: [0, 0], maxForce: 1000, hertz: 4, dampingRatio: 0.7 });
  assert.equal(w.getJointType(mouse), JointType.Motor, "mouse is a motor joint");
  steps(w, 180);
  near(pos(w, body)[0], 2, 0.2, "mouse pulls to target");
  assert.equal(w.setMouseJointTarget(mouse, [-2, 0]), true, "target moved");
  steps(w, 180);
  near(pos(w, body)[0], -2, 0.2, "mouse follows moved target");
  const filter = w.createFilterJoint({ bodyA: ground, bodyB: body });
  assert.equal(w.setMouseJointTarget(filter, [0, 0]), false, "setter refuses a filter joint");
  w.destroy();
  console.log("ok ts joints: mouse");
}

{
  // Filter joint: a box joined to the floor falls through it. Without the joint it rests on the floor.
  const w = world();
  const floorBody = staticBody(w, 0, 0);
  w.createBoxShape(floorBody, { halfWidth: 10, halfHeight: 0.5, density: 1, friction: 0.5 });
  const falling = dynamicBox(w, 0, 1, 0.5);
  const f = w.createFilterJoint({ bodyA: floorBody, bodyB: falling });
  assert.equal(w.getJointType(f), JointType.Filter, "filter kind");
  assert.equal(w.setJointLimits(f, [0, 1]), false, "filter has no limit");
  assert.equal(w.enableJointSpring(f, true), false, "filter has no spring");
  assert.equal(w.enableJointMotor(f, true), false, "filter has no motor");
  steps(w, 120);
  assert.ok(pos(w, falling)[1] < -1, "filter joint lets the box fall through");
  w.destroy();

  const w2 = world();
  const floor2 = staticBody(w2, 0, 0);
  w2.createBoxShape(floor2, { halfWidth: 10, halfHeight: 0.5, density: 1, friction: 0.5 });
  const resting = dynamicBox(w2, 0, 1, 0.5);
  steps(w2, 120);
  near(pos(w2, resting)[1], 1, 0.05, "without the filter the box rests on the floor");
  w2.destroy();
  console.log("ok ts joints: filter");
}

{
  // Reaction: a box hanging from a rigid rod carries its weight (mass 1, g 10): 10 N, no torque.
  const w = world();
  const ceiling = staticBody(w, 0, 2);
  const hanging = dynamicBox(w, 0, 0, 0.5);
  const rod = w.createDistanceJoint({ bodyA: ceiling, bodyB: hanging, length: 2, hertz: 0, dampingRatio: 0 });
  steps(w, 240);
  const r = w.getJointReaction(rod);
  assert.ok(r !== null, "reaction of a valid joint");
  const magnitude = Math.hypot(r.force[0], r.force[1]);
  near(magnitude, 10, 0.5, "rod pulls with the weight");
  assert.ok(Math.abs(r.torque) <= 0.5, `no torque, got ${r.torque}`);
  w.destroy();
  console.log("ok ts joints: reaction");
}

// ---- Queries ----

{
  // Scene: static box A at origin, dynamic circle B at x=5, static box C at x=10, static box D at y=10. No gravity.
  const w = world(0);
  const a = w.createBoxShape(staticBody(w, 0, 0), { halfWidth: 1, halfHeight: 1, density: 1, friction: 0.5 });
  const { shape: b } = dynamicCircle(w, 5, 0);
  const c = w.createBoxShape(staticBody(w, 10, 0), { halfWidth: 1, halfHeight: 1, density: 1, friction: 0.5 });
  w.createBoxShape(staticBody(w, 0, 10), { halfWidth: 1, halfHeight: 1, density: 1, friction: 0.5 });

  const small = w.overlapAABB([-2, -2], [2, 2]);
  assert.equal(small.total, 1, "small box overlaps one shape");
  assert.deepEqual(small.shapes, [a], "and it is A");
  const strip = w.overlapAABB([-6, -1], [6, 1]);
  assert.equal(strip.total, 2, "strip overlaps two shapes");
  assert.ok(strip.shapes.includes(a) && strip.shapes.includes(b), "strip hits A and B");
  const capped = w.overlapAABB([-6, -1], [6, 1], 1);
  assert.equal(capped.total, 2, "total is reported past the result cap");
  assert.equal(capped.shapes.length, 1, "shapes is capped");

  const circle = w.overlapCircle([5, 0], 0.1);
  assert.deepEqual(circle.shapes, [b], "circle inside B");
  assert.equal(w.overlapCircle([0, 3], 0.5).total, 0, "circle clear of everything");

  const tri = w.overlapPolygon([[4, -1], [6, -1], [5, 1]], 0);
  assert.equal(tri.total, 1, "triangle around B");
  assert.equal(tri.shapes[0], b, "triangle hits B");
  assert.throws(() => w.overlapPolygon([], 0), RangeError, "empty proxy rejected");
  assert.throws(() => w.overlapPolygon(Array.from({ length: 9 }, () => [0, 0]), 0), RangeError, "9-point proxy rejected");

  // Ray, all hits: from x=-5 towards +x it crosses A (t=0.2), B (t=0.475) and C (t=0.7).
  const rays = w.castRayAll([-5, 0], [20, 0]);
  assert.equal(rays.total, 3, "ray hits three shapes");
  const found = [...rays.hits].sort((p, q) => p.fraction - q.fraction);
  assert.equal(found[0].shape, a, "first hit is A");
  near(found[0].fraction, 0.2, 1e-4, "A fraction");
  near(found[0].normal[0], -1, 1e-4, "A normal faces the ray");
  assert.equal(found[1].shape, b, "second hit is B");
  near(found[1].fraction, 0.475, 1e-3, "B fraction");
  assert.equal(found[2].shape, c, "third hit is C");
  near(found[2].fraction, 0.7, 1e-3, "C fraction");
  const twoOnly = w.castRayAll([-5, 0], [20, 0], 2);
  assert.equal(twoOnly.total, 3, "total past the hit cap");
  assert.equal(twoOnly.hits.length, 2, "hits capped");

  // Shape cast: a radius 0.5 circle moved from x=-5 meets A's left face at t=0.175.
  const hit = w.castShapeClosest([[0, 0]], 0.5, [-5, 0], [20, 0]);
  assert.ok(hit !== null, "shape cast hits A");
  assert.equal(hit.shape, a, "closest is A");
  near(hit.fraction, 0.175, 1e-3, "shape cast fraction");
  near(hit.normal[0], -1, 1e-3, "shape cast normal");
  assert.equal(w.castShapeClosest([[0, 0]], 0.5, [-5, 5], [20, 0]), null, "passing above misses");
  assert.throws(() => w.castShapeClosest([], 0.5, [-5, 0], [20, 0]), RangeError, "empty proxy rejected");
  w.destroy();

  // Explosion at the origin, radius 5, falloff 2: full impulse at surface distance 1.5, 0.75 at 5.5, none at 7.5.
  const w2 = world(0);
  const near2 = dynamicCircle(w2, 2, 0).body;
  const mid = dynamicCircle(w2, 6, 0).body;
  const far = dynamicCircle(w2, 8, 0).body;
  w2.explode([0, 0], 5, 2, 1);
  const vn = vel(w2, near2);
  const vm = vel(w2, mid);
  assert.ok(vn[0] > 0.1 && Math.abs(vn[1]) <= 1e-5, "inside the radius: pushed outwards");
  near(vm[0] / vn[0], 0.75, 1e-3, "falloff band scales the impulse");
  assert.ok(Math.abs(vm[1]) <= 1e-5, "explosion stays radial");
  assert.deepEqual(vel(w2, far), [0, 0], "beyond radius + falloff: untouched");
  w2.explode([0, 0], 5, 2, 1, ALL_BITS);
  w2.destroy();
  console.log("ok ts queries: shape cast, overlaps, ray cast all, explosion");
}

// ---- Bodies ----

{
  // Linear damping: a damped body slows down, an undamped one keeps its speed.
  const w = world(0);
  const free = dynamicCircle(w, 0, 0).body;
  const damped = dynamicCircle(w, 0, 5).body;
  w.setLinearDamping(damped, 2);
  assert.equal(w.getLinearDamping(damped), 2, "damping round trip");
  assert.equal(w.getLinearDamping(free), 0, "default damping is zero");
  w.setLinearVelocity(free, [10, 0]);
  w.setLinearVelocity(damped, [10, 0]);
  steps(w, 60);
  near(vel(w, free)[0], 10, 1e-3, "undamped keeps speed");
  const dv = vel(w, damped)[0];
  assert.ok(dv > 1 && dv < 2, `damped body slows to about 10 e^-2, got ${dv}`);
  w.destroy();

  // Gravity scale: 1 falls at g, 0 floats.
  const g = world(-10);
  const normal = dynamicCircle(g, 0, 100).body;
  const floating = dynamicCircle(g, 5, 100).body;
  g.setGravityScale(floating, 0);
  assert.equal(g.getGravityScale(floating), 0, "gravity scale round trip");
  assert.equal(g.getGravityScale(normal), 1, "default gravity scale");
  steps(g, 30);
  near(vel(g, normal)[1], -5, 0.1, "normal body falls at g");
  assert.ok(Math.abs(vel(g, floating)[1]) <= 1e-6, "floating body does not fall");
  g.destroy();

  // Fixed rotation: the locked body keeps angle and spin at zero, its linear motion is unaffected.
  const r = world(0);
  const spin = dynamicBox(r, 0, 0, 0.5);
  const locked = dynamicBox(r, 10, 0, 0.5);
  r.setAngularVelocity(spin, 5);
  r.setAngularVelocity(locked, 5);
  r.setLinearVelocity(locked, [3, 0]);
  r.setFixedRotation(locked, true);
  assert.equal(r.isFixedRotation(locked), true, "locked");
  assert.equal(r.isFixedRotation(spin), false, "free");
  steps(r, 60);
  assert.ok(r.getAngularVelocity(spin) >= 4.5, "free body keeps spinning");
  assert.ok(Math.abs(r.getAngularVelocity(locked)) <= 1e-6, "locked body has no spin");
  assert.ok(Math.abs(r.getTransform(locked).rotation) <= 1e-6, "locked body keeps its angle");
  assert.ok(pos(r, locked)[0] >= 12.5, "locked body still moves in x");
  r.setFixedRotation(locked, false);
  assert.equal(r.isFixedRotation(locked), false, "unlocked");
  r.setAngularVelocity(locked, 2);
  steps(r, 1);
  assert.ok(r.getAngularVelocity(locked) >= 1.5, "unlocked body spins again");
  r.destroy();

  // Mass data: a 1 m box of density 2 has mass 2 and inertia m (w^2 + h^2) / 12 = 1/3, then overrides stick.
  const m = world();
  const mb = dynamicBox(m, 0, 0, 0.5, 2);
  const md = m.getMassData(mb);
  near(md.mass, 2, 1e-4, "mass");
  near(md.rotationalInertia, 1 / 3, 1e-4, "rotational inertia");
  near(md.center[0], 0, 1e-6, "center x");
  near(md.center[1], 0, 1e-6, "center y");
  m.setMassData(mb, { mass: 5, center: [0.25, 0], rotationalInertia: 7 });
  const got = m.getMassData(mb);
  near(got.mass, 5, 1e-5, "set mass");
  near(got.center[0], 0.25, 1e-5, "set center");
  near(got.rotationalInertia, 7, 1e-5, "set inertia");
  m.destroy();

  // User data: 0 by default, then whatever was stored, including the full u32 range.
  const u = world();
  const ub = dynamicCircle(u, 0, 0).body;
  assert.equal(u.getBodyUserData(ub), 0, "default user data");
  u.setBodyUserData(ub, 4242);
  assert.equal(u.getBodyUserData(ub), 4242, "user data round trip");
  u.setBodyUserData(ub, 0xffffffff);
  assert.equal(u.getBodyUserData(ub), 4294967295, "full u32 range");
  assert.throws(() => u.setBodyUserData(ub, -1), RangeError, "negative user data rejected");
  u.destroy();

  // Enable and disable: a disabled body does not move, and falls again once enabled.
  const e = world();
  const eb = dynamicCircle(e, 0, 10).body;
  const before = pos(e, eb);
  e.disableBody(eb);
  assert.equal(e.isBodyEnabled(eb), false, "disabled");
  steps(e, 30);
  assert.deepEqual(pos(e, eb), before, "disabled body stays put");
  e.enableBody(eb);
  assert.equal(e.isBodyEnabled(eb), true, "enabled again");
  steps(e, 30);
  assert.ok(before[1] - pos(e, eb)[1] >= 1.0, "falls once enabled");
  e.destroy();

  // Batch impulses: v = J / m for linear, w = J / I for angular, each body gets its own entry.
  const bi = world(0);
  const ids = [0, 5, 10].map((x) => dynamicCircle(bi, x, 0).body);
  const linear = [[1, 0], [0, 2], [-3, 4]];
  bi.applyLinearImpulseBatch(ids, linear.flat(), true);
  ids.forEach((id, i) => {
    const mass = bi.getMassData(id).mass;
    const v = vel(bi, id);
    for (let k = 0; k < 2; k++) {
      const expected = linear[i][k] / mass;
      assert.ok(Math.abs(v[k] - expected) <= 1e-4 * Math.max(Math.abs(expected), 1), `batch linear impulse body ${i}`);
    }
  });
  const angular = [0.5, -0.5, 0];
  bi.applyAngularImpulseBatch(ids, angular, true);
  ids.forEach((id, i) => {
    const inertia = bi.getMassData(id).rotationalInertia;
    const expected = angular[i] / inertia;
    const got = bi.getAngularVelocity(id);
    assert.ok(Math.abs(got - expected) <= 1e-4 * Math.max(Math.abs(expected), 1), `batch angular impulse body ${i}`);
  });
  bi.applyLinearImpulseBatch([], []);
  assert.throws(() => bi.applyLinearImpulseBatch(ids, [1]), Error, "short impulse array rejected");
  bi.destroy();
  console.log("ok ts bodies: damping, gravity scale, fixed rotation, mass data, user data, enable, batch impulses");
}

// ---- Filters ----

{
  // Floors at x = 0 and x = 10 and x = 20, each 6 m wide, top at y = 0.5. Floor A collides only with category 2.
  // Floor C and its ball share group -1, so they pass through each other. Ball B hits floor B.
  const w = world();
  const shapeA = w.createBoxShape(staticBody(w, 0, 0), { halfWidth: 3, halfHeight: 0.5, density: 1, friction: 0.5 });
  w.createBoxShape(staticBody(w, 10, 0), { halfWidth: 3, halfHeight: 0.5, density: 1, friction: 0.5 });
  const shapeC = w.createBoxShape(staticBody(w, 20, 0), { halfWidth: 3, halfHeight: 0.5, density: 1, friction: 0.5 });

  assert.deepEqual(w.getShapeFilter(shapeA), { categoryBits: 1n, maskBits: ALL_BITS, groupIndex: 0 }, "default filter");
  w.setShapeFilter(shapeA, { categoryBits: 2n, maskBits: 2n, groupIndex: 0 });
  w.setShapeFilter(shapeC, { categoryBits: 1n, maskBits: ALL_BITS, groupIndex: -1 });
  assert.deepEqual(w.getShapeFilter(shapeA), { categoryBits: 2n, maskBits: 2n, groupIndex: 0 }, "floor A filter");
  assert.equal(w.getShapeFilter(shapeC).groupIndex, -1, "floor C group");

  const ballA = dynamicCircle(w, 0, 3).body;
  const ballB = dynamicCircle(w, 10, 3).body;
  const circleC = dynamicCircle(w, 20, 3);
  const ballC = circleC.body;
  w.setShapeFilter(circleC.shape, { categoryBits: 1n, maskBits: ALL_BITS, groupIndex: -1 });
  steps(w, 120);
  assert.ok(pos(w, ballA)[1] < -5, "ball A falls through floor A");
  assert.ok(pos(w, ballC)[1] < -5, "ball C falls through floor C (same negative group)");
  const yB = pos(w, ballB)[1];
  assert.ok(yB > 0.9 && yB < 1.2, `ball B rests on floor B, got ${yB}`);
  w.destroy();
  console.log("ok ts filters: category, mask and group");
}

console.log("ok ts wrapper: all new methods exercised");
