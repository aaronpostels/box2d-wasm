// SPDX-License-Identifier: MIT

export type WorldId = number;
export type BodyId = bigint;
export type ShapeId = bigint;
export type JointId = bigint;

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

    // Snapshot
    _b2Glue_SnapshotSize(worldId: number): number;
    _b2Glue_SnapshotTake(worldId: number, outBufferPtr: number, capacity: number): number;
    _b2Glue_SnapshotRestore(worldId: number, bufferPtr: number, size: number): number;
    _b2Glue_SnapshotCreateWorld(bufferPtr: number, size: number, workerCount: number): number;
}
