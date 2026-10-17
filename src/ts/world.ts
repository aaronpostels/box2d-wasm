// SPDX-License-Identifier: MIT

import {
    RawBox2DModule,
    WorldId,
    BodyId,
    ShapeId,
    BodyType,
    BodyTypeValue,
    Vec2Tuple,
    Transform2D,
    CircleShapeOptions,
    BoxShapeOptions,
    CapsuleShapeOptions,
    SegmentShapeOptions,
    PolygonShapeOptions,
    RayHit2D,
    JointId,
    WeldJointOptions,
    RevoluteJointOptions,
    PrismaticJointOptions,
    DistanceJointOptions,
    MotorJointOptions,
    ContactPair,
    SensorPair,
    CharacterMoveResult
} from "./types.js";

export class Box2DWorld {
    private static readonly EMPTY_FLOAT32 = new Float32Array(0);
    private static readonly MAX_BATCH_COUNT = 0x03ffffff;

    private readonly wasm: RawBox2DModule;
    private readonly worldId: WorldId;
    private isDestroyed = false;

    private scratchPtr: number;
    private batchIdsPtr = 0;
    private batchFloatsPtr = 0;
    private batchCapacity = 0;
    private cachedWasmFloatView: Float32Array | null = null;

    constructor(wasm: RawBox2DModule, gravity: Vec2Tuple = [0, -9.81], options: { sleep?: boolean; continuous?: boolean } = {}) {
        this.wasm = wasm;
        this.worldId = wasm._b2Glue_CreateWorld(gravity[0], gravity[1], (options.sleep ?? true) ? 1 : 0, (options.continuous ?? true) ? 1 : 0);
        if (wasm._b2Glue_WorldIsValid(this.worldId) === 0) {
            throw new Error("Box2DWorld: failed to create simulation world.");
        }
        this.scratchPtr = wasm._malloc(128);
        if (!this.scratchPtr) {
            wasm._b2Glue_DestroyWorld(this.worldId);
            throw new Error("Box2DWorld: failed to allocate scratch buffer.");
        }
    }

    public get id(): WorldId {
        return this.worldId;
    }

    public step(timeStep: number = 1 / 60, subStepCount: number = 4): void {
        this.checkDestroyed();
        this.wasm._b2Glue_Step(this.worldId, timeStep, subStepCount);
    }

    public createBody(options: {
        type?: BodyTypeValue;
        position?: Vec2Tuple;
        rotation?: number;
    } = {}): BodyId {
        this.checkDestroyed();
        const type = options.type ?? BodyType.Dynamic;
        const [px, py] = options.position ?? [0, 0];
        const rot = options.rotation ?? 0;
        return this.wasm._b2Glue_CreateBody(this.worldId, type, px, py, rot);
    }

    public destroyBody(bodyId: BodyId): void {
        this.checkDestroyed();
        this.wasm._b2Glue_DestroyBody(bodyId);
    }

    public isBodyValid(bodyId: BodyId): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_BodyIsValid(bodyId) !== 0;
    }

    public createCircleShape(bodyId: BodyId, options: CircleShapeOptions): ShapeId {
        this.checkDestroyed();
        const flags = options.sensor ? 1 : 0;
        return this.wasm._b2Glue_AddCircle(
            bodyId,
            options.radius,
            options.density ?? 1.0,
            options.friction ?? 0.6,
            options.restitution ?? 0.0,
            flags
        );
    }

    public createBoxShape(bodyId: BodyId, options: BoxShapeOptions): ShapeId {
        this.checkDestroyed();
        const flags = options.sensor ? 1 : 0;
        return this.wasm._b2Glue_AddBox(
            bodyId,
            options.halfWidth,
            options.halfHeight,
            options.density ?? 1.0,
            options.friction ?? 0.6,
            options.restitution ?? 0.0,
            flags
        );
    }

    public createCapsuleShape(bodyId: BodyId, options: CapsuleShapeOptions): ShapeId {
        this.checkDestroyed();
        const flags = options.sensor ? 1 : 0;
        return this.wasm._b2Glue_AddCapsule(
            bodyId,
            options.halfHeight,
            options.radius,
            options.density ?? 1.0,
            options.friction ?? 0.6,
            options.restitution ?? 0.0,
            flags
        );
    }

    public createSegmentShape(bodyId: BodyId, options: SegmentShapeOptions): ShapeId {
        this.checkDestroyed();
        const flags = options.sensor ? 1 : 0;
        return this.wasm._b2Glue_AddSegment(
            bodyId,
            options.x1,
            options.y1,
            options.x2,
            options.y2,
            options.density ?? 1.0,
            options.friction ?? 0.6,
            options.restitution ?? 0.0,
            flags
        );
    }

    public createPolygonShape(bodyId: BodyId, options: PolygonShapeOptions): ShapeId {
        this.checkDestroyed();
        const count = options.points.length;
        if (count < 3) throw new Error("Box2DWorld.createPolygonShape: polygon requires at least 3 points.");
        const floats = count * 2;
        let bufPtr = this.scratchPtr;
        let allocated = 0;
        if (floats * 4 > 128) {
            allocated = this.wasm._malloc(floats * 4);
            bufPtr = allocated;
        }
        const f32 = this.getFloat32View();
        const offset = bufPtr >> 2;
        for (let i = 0; i < count; i++) {
            f32[offset + i * 2] = options.points[i][0];
            f32[offset + i * 2 + 1] = options.points[i][1];
        }
        const flags = options.sensor ? 1 : 0;
        const shapeId = this.wasm._b2Glue_AddPolygon(
            bodyId,
            bufPtr,
            count,
            options.radius ?? 0.0,
            options.density ?? 1.0,
            options.friction ?? 0.6,
            options.restitution ?? 0.0,
            flags
        );
        if (allocated) this.wasm._free(allocated);
        return shapeId;
    }

    public destroyShape(shapeId: ShapeId, updateMass: boolean = true): void {
        this.checkDestroyed();
        this.wasm._b2Glue_DestroyShape(shapeId, updateMass);
    }

    public setTransform(bodyId: BodyId, position: Vec2Tuple, rotation: number = 0): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetTransform(bodyId, position[0], position[1], rotation);
    }

    public getTransform(bodyId: BodyId): Transform2D {
        this.checkDestroyed();
        this.wasm._b2Glue_GetTransform(bodyId, this.scratchPtr);
        const f32 = this.getFloat32View();
        const idx = this.scratchPtr >> 2;
        return {
            position: [f32[idx], f32[idx + 1]],
            rotation: f32[idx + 2]
        };
    }

    public setLinearVelocity(bodyId: BodyId, velocity: Vec2Tuple): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetLinearVelocity(bodyId, velocity[0], velocity[1]);
    }

    public getLinearVelocity(bodyId: BodyId): Vec2Tuple {
        this.checkDestroyed();
        this.wasm._b2Glue_GetLinearVelocity(bodyId, this.scratchPtr);
        const f32 = this.getFloat32View();
        const idx = this.scratchPtr >> 2;
        return [f32[idx], f32[idx + 1]];
    }

    public setAngularVelocity(bodyId: BodyId, omega: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetAngularVelocity(bodyId, omega);
    }

    public getAngularVelocity(bodyId: BodyId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_GetAngularVelocity(bodyId);
    }

    public applyForce(bodyId: BodyId, force: Vec2Tuple, point: Vec2Tuple, wake: boolean = true): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ApplyForce(bodyId, force[0], force[1], point[0], point[1], wake);
    }

    public applyLinearImpulse(bodyId: BodyId, impulse: Vec2Tuple, point: Vec2Tuple, wake: boolean = true): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ApplyLinearImpulse(bodyId, impulse[0], impulse[1], point[0], point[1], wake);
    }

    public applyAngularImpulse(bodyId: BodyId, impulse: number, wake: boolean = true): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ApplyAngularImpulse(bodyId, impulse, wake);
    }

    public applyForceToCenter(bodyId: BodyId, force: Vec2Tuple, wake: boolean = true): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ApplyForceToCenter(bodyId, force[0], force[1], wake);
    }

    public applyLinearImpulseToCenter(bodyId: BodyId, impulse: Vec2Tuple, wake: boolean = true): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ApplyLinearImpulseToCenter(bodyId, impulse[0], impulse[1], wake);
    }

    public getBodyMass(bodyId: BodyId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_BodyMass(bodyId);
    }

    public setBodyMass(bodyId: BodyId, mass: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetBodyMass(bodyId, mass);
    }

    public isBodyAwake(bodyId: BodyId): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_BodyIsAwake(bodyId) !== 0;
    }

    public getStateHash(): bigint {
        this.checkDestroyed();
        return this.wasm._b2Glue_StateHash(this.worldId);
    }

    public syncBatchTransforms(bodyIds: ArrayLike<BodyId>, out: Float32Array): void {
        this.checkDestroyed();
        const count = bodyIds.length;
        if (count === 0) return;
        if (out.length < count * 3) {
            throw new Error(`Box2DWorld: destination array too small (length=${out.length}, needed=${count * 3}).`);
        }
        this.ensureBatchCapacity(count, 3);
        const u64 = this.getBigUint64View();
        const idOffset = this.batchIdsPtr >> 3;
        for (let i = 0; i < count; i++) {
            u64[idOffset + i] = bodyIds[i];
        }
        this.wasm._b2Glue_GetBatchTransforms(this.batchIdsPtr, count, this.batchFloatsPtr);
        const f32 = this.getFloat32View();
        const floatOffset = this.batchFloatsPtr >> 2;
        out.set(f32.subarray(floatOffset, floatOffset + count * 3));
    }

    public castRayClosest(origin: Vec2Tuple, translation: Vec2Tuple): RayHit2D {
        this.checkDestroyed();
        const shapeId = this.wasm._b2Glue_CastRayClosest(this.worldId, this.scratchPtr, this.scratchPtr + 8, this.scratchPtr + 16);
        const f32 = this.getFloat32View();
        const idx = (this.scratchPtr + 16) >> 2;
        return {
            hit: shapeId !== 0n && f32[idx + 5] > 0.0,
            fraction: f32[idx],
            point: [f32[idx + 1], f32[idx + 2]],
            normal: [f32[idx + 3], f32[idx + 4]]
        };
    }

    public createWeldJoint(options: WeldJointOptions): JointId {
        this.checkDestroyed();
        const a = options.anchorA ?? [0, 0];
        const b = options.anchorB ?? [0, 0];
        const f32 = this.getFloat32View();
        const idx = this.scratchPtr >> 2;
        f32[idx] = a[0];
        f32[idx + 1] = a[1];
        f32[idx + 2] = b[0];
        f32[idx + 3] = b[1];
        return this.wasm._b2Glue_CreateWeld(
            this.worldId,
            options.bodyA,
            options.bodyB,
            this.scratchPtr,
            this.scratchPtr + 8,
            options.linearHertz ?? 0.0,
            options.angularHertz ?? 0.0
        );
    }

    public createRevoluteJoint(options: RevoluteJointOptions): JointId {
        this.checkDestroyed();
        const a = options.anchorA ?? [0, 0];
        const b = options.anchorB ?? [0, 0];
        const f32 = this.getFloat32View();
        const idx = this.scratchPtr >> 2;
        f32[idx] = a[0];
        f32[idx + 1] = a[1];
        f32[idx + 2] = b[0];
        f32[idx + 3] = b[1];
        const lim = options.limits;
        return this.wasm._b2Glue_CreateRevolute(
            this.worldId,
            options.bodyA,
            options.bodyB,
            this.scratchPtr,
            this.scratchPtr + 8,
            lim ? 1 : 0,
            lim ? lim[0] : 0,
            lim ? lim[1] : 0
        );
    }

    public createPrismaticJoint(options: PrismaticJointOptions): JointId {
        this.checkDestroyed();
        const a = options.anchorA ?? [0, 0];
        const b = options.anchorB ?? [0, 0];
        const ax = options.axis ?? [1, 0];
        const f32 = this.getFloat32View();
        const idx = this.scratchPtr >> 2;
        f32[idx] = a[0];
        f32[idx + 1] = a[1];
        f32[idx + 2] = b[0];
        f32[idx + 3] = b[1];
        f32[idx + 4] = ax[0];
        f32[idx + 5] = ax[1];
        const lim = options.limits;
        return this.wasm._b2Glue_CreatePrismatic(
            this.worldId,
            options.bodyA,
            options.bodyB,
            this.scratchPtr,
            this.scratchPtr + 8,
            this.scratchPtr + 16,
            lim ? 1 : 0,
            lim ? lim[0] : 0,
            lim ? lim[1] : 0
        );
    }

    public createDistanceJoint(options: DistanceJointOptions): JointId {
        this.checkDestroyed();
        const a = options.anchorA ?? [0, 0];
        const b = options.anchorB ?? [0, 0];
        const f32 = this.getFloat32View();
        const idx = this.scratchPtr >> 2;
        f32[idx] = a[0];
        f32[idx + 1] = a[1];
        f32[idx + 2] = b[0];
        f32[idx + 3] = b[1];
        return this.wasm._b2Glue_CreateDistance(
            this.worldId,
            options.bodyA,
            options.bodyB,
            this.scratchPtr,
            this.scratchPtr + 8,
            options.length ?? 1.0,
            options.hertz ?? 0.0,
            options.dampingRatio ?? 0.7
        );
    }

    public createMotorJoint(options: MotorJointOptions): JointId {
        this.checkDestroyed();
        return this.wasm._b2Glue_CreateMotor(
            this.worldId,
            options.bodyA,
            options.bodyB,
            options.maxForce ?? 1000.0,
            options.maxTorque ?? 1000.0
        );
    }

    public destroyJoint(jointId: JointId): void {
        this.checkDestroyed();
        this.wasm._b2Glue_DestroyJoint(jointId);
    }

    public isJointValid(jointId: JointId): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_JointIsValid(jointId) !== 0;
    }

    public moveCharacter(
        selfShapeId: ShapeId,
        position: Vec2Tuple,
        halfHeight: number,
        radius: number,
        delta: Vec2Tuple,
        groundNormalY: number = 0.7
    ): CharacterMoveResult {
        this.checkDestroyed();
        const f32 = this.getFloat32View();
        const idx = this.scratchPtr >> 2;
        f32[idx] = position[0];
        f32[idx + 1] = position[1];
        f32[idx + 2] = delta[0];
        f32[idx + 3] = delta[1];
        const grounded = this.wasm._b2Glue_MoveCharacter(
            this.worldId,
            selfShapeId,
            this.scratchPtr,
            halfHeight,
            radius,
            this.scratchPtr + 8,
            groundNormalY
        );
        return {
            position: [f32[idx], f32[idx + 1]],
            grounded: grounded !== 0
        };
    }

    public getContactBeginEvents(maxEvents: number = 64): ContactPair[] {
        this.checkDestroyed();
        const buf = this.wasm._malloc(maxEvents * 16);
        const count = this.wasm._b2Glue_ContactBeginEvents(this.worldId, buf, maxEvents);
        const u64 = this.getBigUint64View();
        const offset = buf >> 3;
        const res: ContactPair[] = [];
        for (let i = 0; i < count; i++) {
            res.push({
                shapeA: u64[offset + i * 2],
                shapeB: u64[offset + i * 2 + 1]
            });
        }
        this.wasm._free(buf);
        return res;
    }

    public getContactEndEvents(maxEvents: number = 64): ContactPair[] {
        this.checkDestroyed();
        const buf = this.wasm._malloc(maxEvents * 16);
        const count = this.wasm._b2Glue_ContactEndEvents(this.worldId, buf, maxEvents);
        const u64 = this.getBigUint64View();
        const offset = buf >> 3;
        const res: ContactPair[] = [];
        for (let i = 0; i < count; i++) {
            res.push({
                shapeA: u64[offset + i * 2],
                shapeB: u64[offset + i * 2 + 1]
            });
        }
        this.wasm._free(buf);
        return res;
    }

    public getSensorBeginEvents(maxEvents: number = 64): SensorPair[] {
        this.checkDestroyed();
        const buf = this.wasm._malloc(maxEvents * 16);
        const count = this.wasm._b2Glue_SensorBeginEvents(this.worldId, buf, maxEvents);
        const u64 = this.getBigUint64View();
        const offset = buf >> 3;
        const res: SensorPair[] = [];
        for (let i = 0; i < count; i++) {
            res.push({
                sensorShape: u64[offset + i * 2],
                visitorShape: u64[offset + i * 2 + 1]
            });
        }
        this.wasm._free(buf);
        return res;
    }

    public getSensorEndEvents(maxEvents: number = 64): SensorPair[] {
        this.checkDestroyed();
        const buf = this.wasm._malloc(maxEvents * 16);
        const count = this.wasm._b2Glue_SensorEndEvents(this.worldId, buf, maxEvents);
        const u64 = this.getBigUint64View();
        const offset = buf >> 3;
        const res: SensorPair[] = [];
        for (let i = 0; i < count; i++) {
            res.push({
                sensorShape: u64[offset + i * 2],
                visitorShape: u64[offset + i * 2 + 1]
            });
        }
        this.wasm._free(buf);
        return res;
    }

    public destroy(): void {
        if (this.isDestroyed) return;
        if (this.scratchPtr) {
            this.wasm._free(this.scratchPtr);
            this.scratchPtr = 0;
        }
        if (this.batchCapacity > 0) {
            this.wasm._free(this.batchIdsPtr);
            this.wasm._free(this.batchFloatsPtr);
            this.batchIdsPtr = 0;
            this.batchFloatsPtr = 0;
            this.batchCapacity = 0;
        }
        this.wasm._b2Glue_DestroyWorld(this.worldId);
        this.isDestroyed = true;
    }

    private checkDestroyed(): void {
        if (this.isDestroyed) throw new Error("Box2DWorld instance has already been destroyed.");
    }

    private getFloat32View(): Float32Array {
        const heap = this.wasm.HEAPF32;
        if (!this.cachedWasmFloatView || this.cachedWasmFloatView.buffer !== heap.buffer) {
            this.cachedWasmFloatView = heap;
        }
        return this.cachedWasmFloatView;
    }

    private getBigUint64View(): BigUint64Array {
        return this.wasm.HEAPU64;
    }

    private ensureBatchCapacity(count: number, floatsPerBody: number = 3): void {
        if (count > Box2DWorld.MAX_BATCH_COUNT) {
            throw new Error(`Box2DWorld: batch count exceeds maximum allowed (${Box2DWorld.MAX_BATCH_COUNT}).`);
        }
        if (count <= this.batchCapacity) return;
        let newCapacity = this.batchCapacity === 0 ? 128 : this.batchCapacity;
        while (newCapacity < count) newCapacity <<= 1;
        if (this.batchCapacity > 0) {
            this.wasm._free(this.batchIdsPtr);
            this.wasm._free(this.batchFloatsPtr);
        }
        this.batchIdsPtr = this.wasm._malloc(newCapacity * 8);
        this.batchFloatsPtr = this.wasm._malloc(newCapacity * floatsPerBody * 4);
        if (!this.batchIdsPtr || !this.batchFloatsPtr) {
            throw new Error("Box2DWorld: failed to allocate memory for batch operation.");
        }
        this.batchCapacity = newCapacity;
    }
}
