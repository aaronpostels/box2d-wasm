// SPDX-License-Identifier: MIT

export type WorldId = number;
export type BodyId = bigint;
export type ShapeId = bigint;
export type JointId = bigint;
export type ChainId = bigint;

// b2JointType values, as returned by Box2DWorld.getJointType.
export const JointType = {
    Distance: 0,
    Filter: 1,
    Motor: 2,
    Mover: 3,
    Pogo: 4,
    Prismatic: 5,
    Revolute: 6,
    Weld: 7,
    Wheel: 8,
} as const;

export type JointTypeValue = typeof JointType[keyof typeof JointType];

export const BodyType = {
    Static: 0,
    Kinematic: 1,
    Dynamic: 2,
} as const;

export type BodyTypeValue = typeof BodyType[keyof typeof BodyType];

export type Vec2Tuple = [number, number];

export interface Transform2D {
    position: Vec2Tuple;
    rotation: number; // radians
}

export interface RayHit2D {
    hit: boolean;
    fraction: number;
    point: Vec2Tuple;
    normal: Vec2Tuple;
}

export interface CircleShapeOptions {
    radius: number;
    density?: number;
    friction?: number;
    restitution?: number;
    sensor?: boolean;
}

export interface BoxShapeOptions {
    halfWidth: number;
    halfHeight: number;
    density?: number;
    friction?: number;
    restitution?: number;
    sensor?: boolean;
}

export interface CapsuleShapeOptions {
    halfHeight: number;
    radius: number;
    density?: number;
    friction?: number;
    restitution?: number;
    sensor?: boolean;
}

export interface SegmentShapeOptions {
    x1: number;
    y1: number;
    x2: number;
    y2: number;
    density?: number;
    friction?: number;
    restitution?: number;
    sensor?: boolean;
}

export interface PolygonShapeOptions {
    points: Vec2Tuple[];
    radius?: number;
    density?: number;
    friction?: number;
    restitution?: number;
    sensor?: boolean;
}

export interface WeldJointOptions {
    bodyA: BodyId;
    bodyB: BodyId;
    anchorA?: Vec2Tuple;
    anchorB?: Vec2Tuple;
    linearHertz?: number;
    angularHertz?: number;
}

export interface RevoluteJointOptions {
    bodyA: BodyId;
    bodyB: BodyId;
    anchorA?: Vec2Tuple;
    anchorB?: Vec2Tuple;
    limits?: [number, number];
}

export interface PrismaticJointOptions {
    bodyA: BodyId;
    bodyB: BodyId;
    anchorA?: Vec2Tuple;
    anchorB?: Vec2Tuple;
    axis?: Vec2Tuple;
    limits?: [number, number];
}

export interface DistanceJointOptions {
    bodyA: BodyId;
    bodyB: BodyId;
    anchorA?: Vec2Tuple;
    anchorB?: Vec2Tuple;
    length?: number;
    hertz?: number;
    dampingRatio?: number;
}

export interface MotorJointOptions {
    bodyA: BodyId;
    bodyB: BodyId;
    maxForce?: number;
    maxTorque?: number;
}

export interface ContactPair {
    shapeA: ShapeId;
    shapeB: ShapeId;
}

export interface SensorPair {
    sensorShape: ShapeId;
    visitorShape: ShapeId;
}

export interface CharacterMoveResult {
    position: Vec2Tuple;
    grounded: boolean;
}

export interface ChainShapeOptions {
    points: Vec2Tuple[];
    isLoop?: boolean; // closed loop (3+ points) or open one-sided chain (2+ points)
    friction?: number;
    restitution?: number;
    rollingResistance?: number;
    tangentSpeed?: number;
}

export interface ShapeFilter {
    categoryBits: bigint;
    maskBits: bigint;
    groupIndex: number;
}

export interface MassData {
    mass: number;
    center: Vec2Tuple;
    rotationalInertia: number;
}

// One query hit. Shape ids come back as bigint like every other id.
export interface QueryHit {
    shape: ShapeId;
    fraction: number;
    point: Vec2Tuple;
    normal: Vec2Tuple;
}

// total counts every shape or hit, even when only the first maxResults are returned in shapes or hits.
export interface OverlapResult {
    total: number;
    shapes: ShapeId[];
}

export interface RayAllResult {
    total: number;
    hits: QueryHit[];
}

export interface WheelJointOptions {
    bodyA: BodyId;
    bodyB: BodyId;
    anchorA?: Vec2Tuple;
    anchorB?: Vec2Tuple;
    axis?: Vec2Tuple; // slide axis in frame A, default [1, 0]
    spring?: { hertz: number; dampingRatio: number };
    limits?: [number, number]; // translation limits along the axis
    motor?: { speed: number; maxTorque: number };
}

// Mouse joint: a motor joint with a linear spring pulling anchorB towards target (world). Pass the ground body as bodyA.
export interface MouseJointOptions {
    bodyA: BodyId;
    bodyB: BodyId;
    target: Vec2Tuple;
    anchorB?: Vec2Tuple;
    maxForce?: number;
    hertz?: number;
    dampingRatio?: number;
}

export interface FilterJointOptions {
    bodyA: BodyId;
    bodyB: BodyId;
}

export interface JointReaction {
    force: Vec2Tuple; // on body B, from the last step
    torque: number;
}

export interface RawBox2DModule {
    HEAP8: Int8Array;
    HEAPU8: Uint8Array;
    HEAP32: Int32Array;
    HEAPU32: Uint32Array;
    HEAPF32: Float32Array;
    HEAP64: BigInt64Array;
    HEAPU64: BigUint64Array;

    _malloc(bytes: number): number;
    _free(ptr: number): void;

    // World
    _b2Glue_CreateWorld(gx: number, gy: number, enableSleep: number, enableContinuous: number): number;
    _b2Glue_DestroyWorld(worldId: number): void;
    _b2Glue_Step(worldId: number, timeStep: number, subStepCount: number): void;
    _b2Glue_WorldIsValid(worldId: number): number;
    _b2Glue_SetGravity(worldId: number, gx: number, gy: number): void;
    _b2Glue_Configure(worldId: number, sleeping: boolean | number, continuous: boolean | number): void;
    _b2Glue_SetContactTuning(worldId: number, hertz: number, dampingRatio: number, pushSpeed: number): void;
    _b2Glue_StateHash(worldId: number): bigint;
    _b2Glue_WorkerCount(worldId: number): number;

    // Body
    _b2Glue_CreateBody(worldId: number, type: number, px: number, py: number, rotation: number): bigint;
    _b2Glue_CreateBodyEx(worldId: number, type: number, posePtr3: number, velPtr3: number, flags: number): bigint;
    _b2Glue_DestroyBody(bodyId: bigint): void;
    _b2Glue_BodyIsValid(bodyId: bigint): number;
    _b2Glue_SetTransform(bodyId: bigint, px: number, py: number, rotation: number): void;
    _b2Glue_GetTransform(bodyId: bigint, outPtr3: number): void;
    _b2Glue_SetTargetTransform(bodyId: bigint, posePtr3: number, dt: number): void;
    _b2Glue_SetLinearVelocity(bodyId: bigint, vx: number, vy: number): void;
    _b2Glue_GetLinearVelocity(bodyId: bigint, outPtr2: number): void;
    _b2Glue_SetAngularVelocity(bodyId: bigint, omega: number): void;
    _b2Glue_GetAngularVelocity(bodyId: bigint): number;
    _b2Glue_ApplyForce(bodyId: bigint, fx: number, fy: number, px: number, py: number, wake: boolean | number): void;
    _b2Glue_ApplyForceToCenter(bodyId: bigint, fx: number, fy: number, wake: boolean | number): void;
    _b2Glue_ApplyLinearImpulse(bodyId: bigint, ix: number, iy: number, px: number, py: number, wake: boolean | number): void;
    _b2Glue_ApplyLinearImpulseToCenter(bodyId: bigint, ix: number, iy: number, wake: boolean | number): void;
    _b2Glue_ApplyAngularImpulse(bodyId: bigint, impulse: number, wake: boolean | number): void;
    _b2Glue_BodyMass(bodyId: bigint): number;
    _b2Glue_SetBodyMass(bodyId: bigint, mass: number): void;
    _b2Glue_BodyIsAwake(bodyId: bigint): number;
    _b2Glue_SetSleepThreshold(bodyId: bigint, threshold: number): void;

    // Batch and Events
    _b2Glue_GetBatchTransforms(bodyIdsPtr: number, count: number, outFloatsPtr: number): void;
    _b2Glue_GetBatchState(bodyIdsPtr: number, count: number, outFloatsPtr: number): void;
    _b2Glue_MovedBodies(worldId: number, idsPtr: number, outFloatsPtr: number, max: number): number;

    // Shapes
    _b2Glue_AddCircle(bodyId: bigint, radius: number, density: number, friction: number, restitution: number, flags: number): bigint;
    _b2Glue_AddBox(bodyId: bigint, hx: number, hy: number, density: number, friction: number, restitution: number, flags: number): bigint;
    _b2Glue_AddCapsule(bodyId: bigint, halfHeight: number, radius: number, density: number, friction: number, restitution: number, flags: number): bigint;
    _b2Glue_AddSegment(bodyId: bigint, x1: number, y1: number, x2: number, y2: number, density: number, friction: number, restitution: number, flags: number): bigint;
    _b2Glue_AddPolygon(bodyId: bigint, pointsPtr2: number, count: number, radius: number, density: number, friction: number, restitution: number, flags: number): bigint;
    _b2Glue_DestroyShape(shapeId: bigint, updateMass: boolean | number): void;
    _b2Glue_ShapeIsValid(shapeId: bigint): number;
    _b2Glue_ShapeBody(shapeId: bigint): bigint;

    // Joints
    _b2Glue_CreateWeld(worldId: number, a: bigint, b: bigint, anchorAPtr2: number, anchorBPtr2: number, linearHertz: number, angularHertz: number): bigint;
    _b2Glue_CreateRevolute(worldId: number, a: bigint, b: bigint, anchorAPtr2: number, anchorBPtr2: number, limited: number, lower: number, upper: number): bigint;
    _b2Glue_CreatePrismatic(worldId: number, a: bigint, b: bigint, anchorAPtr2: number, anchorBPtr2: number, axisPtr2: number, limited: number, lower: number, upper: number): bigint;
    _b2Glue_CreateDistance(worldId: number, a: bigint, b: bigint, anchorAPtr2: number, anchorBPtr2: number, length: number, hertz: number, damping: number): bigint;
    _b2Glue_CreateMotor(worldId: number, a: bigint, b: bigint, maxForce: number, maxTorque: number): bigint;
    _b2Glue_DestroyJoint(jointId: bigint): void;
    _b2Glue_JointIsValid(jointId: bigint): number;

    // Events
    _b2Glue_ContactBeginEvents(worldId: number, outPtr2: number, max: number): number;
    _b2Glue_ContactEndEvents(worldId: number, outPtr2: number, max: number): number;
    _b2Glue_SensorBeginEvents(worldId: number, outPtr2: number, max: number): number;
    _b2Glue_SensorEndEvents(worldId: number, outPtr2: number, max: number): number;

    // Raycast & Character
    _b2Glue_CastRayClosest(worldId: number, originPtr2: number, translationPtr2: number, outPtr6: number): bigint;
    _b2Glue_MoveCharacter(worldId: number, selfShapeId: bigint, posPtr2: number, halfHeight: number, radius: number, deltaPtr2: number, groundNormalY: number): number;

    // Shape properties, chains and filters
    _b2Glue_ShapeSetFriction(shapeId: bigint, friction: number): void;
    _b2Glue_ShapeFriction(shapeId: bigint): number;
    _b2Glue_ShapeSetRestitution(shapeId: bigint, restitution: number): void;
    _b2Glue_ShapeRestitution(shapeId: bigint): number;
    _b2Glue_ShapeSetDensity(shapeId: bigint, density: number, updateBodyMass: boolean | number): void;
    _b2Glue_ShapeDensity(shapeId: bigint): number;
    _b2Glue_ShapeIsSensor(shapeId: bigint): number;
    _b2Glue_ShapeSetRollingResistance(shapeId: bigint, rollingResistance: number): void;
    _b2Glue_ShapeRollingResistance(shapeId: bigint): number;
    _b2Glue_ShapeSetTangentSpeed(shapeId: bigint, tangentSpeed: number): void;
    _b2Glue_ShapeTangentSpeed(shapeId: bigint): number;
    _b2Glue_ShapeRadius(shapeId: bigint): number;
    _b2Glue_AddChain(bodyId: bigint, points2: number, count: number, isLoop: boolean | number, friction: number, restitution: number, rollingResistance: number, tangentSpeed: number): bigint;
    _b2Glue_ChainIsValid(chainId: bigint): number;
    _b2Glue_ChainSegmentCount(chainId: bigint): number;
    _b2Glue_SetShapeFilter(shapeId: bigint, categoryBits: bigint, maskBits: bigint, groupIndex: number): void;
    _b2Glue_GetShapeFilter(shapeId: bigint, out2: number, outGroup: number): void;

    // Body extras
    _b2Glue_SetLinearDamping(bodyId: bigint, damping: number): void;
    _b2Glue_GetLinearDamping(bodyId: bigint): number;
    _b2Glue_SetAngularDamping(bodyId: bigint, damping: number): void;
    _b2Glue_GetAngularDamping(bodyId: bigint): number;
    _b2Glue_SetGravityScale(bodyId: bigint, scale: number): void;
    _b2Glue_GetGravityScale(bodyId: bigint): number;
    _b2Glue_SetFixedRotation(bodyId: bigint, fixed: boolean | number): void;
    _b2Glue_GetFixedRotation(bodyId: bigint): number;
    _b2Glue_GetMassData(bodyId: bigint, out4: number): void;
    _b2Glue_SetMassData(bodyId: bigint, in4: number): void;
    _b2Glue_SetBodyUserData(bodyId: bigint, value: number): void;
    _b2Glue_GetBodyUserData(bodyId: bigint): number;
    _b2Glue_EnableBody(bodyId: bigint): void;
    _b2Glue_DisableBody(bodyId: bigint): void;
    _b2Glue_BodyIsEnabled(bodyId: bigint): number;
    _b2Glue_ApplyLinearImpulseBatch(ids: number, count: number, impulses2: number, wake: boolean | number): void;
    _b2Glue_ApplyAngularImpulseBatch(ids: number, count: number, angular1: number, wake: boolean | number): void;

    // Joints: creation, kind, limits, springs, motors, reaction
    _b2Glue_CreateWheel(worldId: number, a: bigint, b: bigint, anchorA2: number, anchorB2: number, axis2: number, enableSpring: boolean | number, hertz: number, dampingRatio: number, enableLimit: boolean | number, lower: number, upper: number, enableMotor: boolean | number, motorSpeed: number, maxMotorTorque: number): bigint;
    _b2Glue_CreateMouse(worldId: number, ground: bigint, b: bigint, target2: number, anchorB2: number, maxForce: number, hertz: number, dampingRatio: number): bigint;
    _b2Glue_CreateFilter(worldId: number, a: bigint, b: bigint): bigint;
    _b2Glue_MouseSetTarget(jointId: bigint, x: number, y: number): number;
    _b2Glue_JointType(jointId: bigint): number;
    _b2Glue_JointSetLimits(jointId: bigint, lower: number, upper: number): number;
    _b2Glue_JointEnableLimit(jointId: bigint, enable: boolean | number): number;
    _b2Glue_JointEnableSpring(jointId: bigint, enable: boolean | number): number;
    _b2Glue_JointSetSpring(jointId: bigint, hertz: number, dampingRatio: number): number;
    _b2Glue_JointEnableMotor(jointId: bigint, enable: boolean | number): number;
    _b2Glue_JointSetMotor(jointId: bigint, speed: number, maxForce: number): number;
    _b2Glue_JointReaction(jointId: bigint, out3: number): number;

    // Queries
    _b2Glue_CastShapeClosest(worldId: number, points2: number, count: number, radius: number, origin2: number, translation2: number, out: number): number;
    _b2Glue_OverlapAABB(worldId: number, minX: number, minY: number, maxX: number, maxY: number, out: number, max: number): number;
    _b2Glue_OverlapCircle(worldId: number, cx: number, cy: number, radius: number, out: number, max: number): number;
    _b2Glue_OverlapPolygon(worldId: number, points2: number, count: number, radius: number, out: number, max: number): number;
    _b2Glue_CastRayAll(worldId: number, origin2: number, translation2: number, out: number, max: number): number;
    _b2Glue_Explode(worldId: number, px: number, py: number, radius: number, falloff: number, impulsePerLength: number, maskBits: bigint): void;

    // Snapshot
    _b2Glue_SnapshotSize(worldId: number): number;
    _b2Glue_SnapshotTake(worldId: number, outBufferPtr: number, capacity: number): number;
    _b2Glue_SnapshotRestore(worldId: number, bufferPtr: number, size: number): number;
    _b2Glue_SnapshotCreateWorld(bufferPtr: number, size: number, workerCount: number): number;
}
