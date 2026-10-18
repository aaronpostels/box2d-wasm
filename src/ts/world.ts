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
    CharacterMoveResult,
    ChainId,
    ChainShapeOptions,
    ShapeFilter,
    MassData,
    QueryHit,
    OverlapResult,
    RayAllResult,
    WheelJointOptions,
    MouseJointOptions,
    FilterJointOptions,
    JointReaction,
    JointTypeValue
} from "./types.js";

const HIT_BYTES = 32; // b2Glue_Hit: u64 shape, f32 fraction, f32 point[2], f32 normal[2]
const MAX_PROXY_POINTS = 8;
const ALL_BITS = 0xffffffffffffffffn;

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

    // ---- Shapes: properties, rounded polygon radius, chains, filters ----

    public getShapeRadius(shapeId: ShapeId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_ShapeRadius(shapeId);
    }

    public getShapeFriction(shapeId: ShapeId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_ShapeFriction(shapeId);
    }

    public setShapeFriction(shapeId: ShapeId, friction: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ShapeSetFriction(shapeId, friction);
    }

    public getShapeRestitution(shapeId: ShapeId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_ShapeRestitution(shapeId);
    }

    public setShapeRestitution(shapeId: ShapeId, restitution: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ShapeSetRestitution(shapeId, restitution);
    }

    public getShapeDensity(shapeId: ShapeId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_ShapeDensity(shapeId);
    }

    /** updateBodyMass re-computes the body's mass and inertia from its shapes (default true). */
    public setShapeDensity(shapeId: ShapeId, density: number, updateBodyMass: boolean = true): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ShapeSetDensity(shapeId, density, updateBodyMass);
    }

    /** Box2D fixes the sensor flag at creation, so there is no setter. */
    public isShapeSensor(shapeId: ShapeId): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_ShapeIsSensor(shapeId) !== 0;
    }

    public getShapeRollingResistance(shapeId: ShapeId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_ShapeRollingResistance(shapeId);
    }

    public setShapeRollingResistance(shapeId: ShapeId, rollingResistance: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ShapeSetRollingResistance(shapeId, rollingResistance);
    }

    public getShapeTangentSpeed(shapeId: ShapeId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_ShapeTangentSpeed(shapeId);
    }

    /** Surface speed along the shape: a belt. Box2D moves touching bodies at this speed along the surface. */
    public setShapeTangentSpeed(shapeId: ShapeId, tangentSpeed: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_ShapeSetTangentSpeed(shapeId, tangentSpeed);
    }

    /**
     * Polyline chain. An open chain needs 2+ points and collides on the right-hand side of each segment only.
     * A loop needs 3+ points: a clockwise loop is solid inside. Returns the chain id (a chain, not a shape).
     */
    public createChainShape(bodyId: BodyId, options: ChainShapeOptions): ChainId {
        this.checkDestroyed();
        const isLoop = options.isLoop ?? false;
        const count = options.points.length;
        const min = isLoop ? 3 : 2;
        if (count < min) {
            throw new Error(`Box2DWorld.createChainShape: ${isLoop ? "a loop" : "an open chain"} needs at least ${min} points.`);
        }
        const flat = options.points.flat();
        return this.withBytes(count * 8, p => {
            this.writeFloats(p, flat);
            const chain = this.wasm._b2Glue_AddChain(
                bodyId,
                p,
                count,
                isLoop ? 1 : 0,
                options.friction ?? 0.6,
                options.restitution ?? 0.0,
                options.rollingResistance ?? 0.0,
                options.tangentSpeed ?? 0.0
            );
            if (chain === 0n) throw new Error("Box2DWorld.createChainShape: Box2D rejected the chain.");
            return chain;
        });
    }

    public isChainValid(chainId: ChainId): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_ChainIsValid(chainId) !== 0;
    }

    public getChainSegmentCount(chainId: ChainId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_ChainSegmentCount(chainId);
    }

    /** Category bits, mask bits and group index. Box2D rebuilds the shape's contacts. */
    public setShapeFilter(shapeId: ShapeId, filter: ShapeFilter): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetShapeFilter(shapeId, filter.categoryBits, filter.maskBits, filter.groupIndex);
    }

    public getShapeFilter(shapeId: ShapeId): ShapeFilter {
        this.checkDestroyed();
        const p = this.scratchPtr;
        this.wasm._b2Glue_GetShapeFilter(shapeId, p, p + 16);
        const u64 = this.getBigUint64View();
        return {
            categoryBits: u64[p >> 3],
            maskBits: u64[(p >> 3) + 1],
            groupIndex: this.wasm.HEAP32[(p + 16) >> 2]
        };
    }

    // ---- Bodies: damping, gravity scale, fixed rotation, mass data, user data, enable, batch impulses ----

    public getLinearDamping(bodyId: BodyId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_GetLinearDamping(bodyId);
    }

    public setLinearDamping(bodyId: BodyId, damping: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetLinearDamping(bodyId, damping);
    }

    public getAngularDamping(bodyId: BodyId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_GetAngularDamping(bodyId);
    }

    public setAngularDamping(bodyId: BodyId, damping: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetAngularDamping(bodyId, damping);
    }

    /** 1 is normal gravity, 0 floats the body. */
    public getGravityScale(bodyId: BodyId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_GetGravityScale(bodyId);
    }

    public setGravityScale(bodyId: BodyId, scale: number): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetGravityScale(bodyId, scale);
    }

    /** Locks the angle and spin of the body; its linear motion is unaffected. */
    public isFixedRotation(bodyId: BodyId): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_GetFixedRotation(bodyId) !== 0;
    }

    public setFixedRotation(bodyId: BodyId, fixed: boolean): void {
        this.checkDestroyed();
        this.wasm._b2Glue_SetFixedRotation(bodyId, fixed);
    }

    public getMassData(bodyId: BodyId): MassData {
        this.checkDestroyed();
        const p = this.scratchPtr;
        this.wasm._b2Glue_GetMassData(bodyId, p);
        const f = this.readFloats(p, 4);
        return { mass: f[0], center: [f[1], f[2]], rotationalInertia: f[3] };
    }

    /** Overrides the mass, center of mass and rotational inertia. Later shape changes recompute them. */
    public setMassData(bodyId: BodyId, data: MassData): void {
        this.checkDestroyed();
        const p = this.scratchPtr;
        this.writeFloats(p, [data.mass, data.center[0], data.center[1], data.rotationalInertia]);
        this.wasm._b2Glue_SetMassData(bodyId, p);
    }

    /** 32-bit user value, for example an entity index. Pointers do not fit, so store a handle. */
    public getBodyUserData(bodyId: BodyId): number {
        this.checkDestroyed();
        return this.wasm._b2Glue_GetBodyUserData(bodyId) >>> 0;
    }

    public setBodyUserData(bodyId: BodyId, value: number): void {
        this.checkDestroyed();
        if (!Number.isInteger(value) || value < 0 || value > 0xffffffff) {
            throw new RangeError("Box2DWorld.setBodyUserData: value must be an integer in 0..4294967295.");
        }
        this.wasm._b2Glue_SetBodyUserData(bodyId, value);
    }

    /** A disabled body takes no part in the simulation and has no contacts. */
    public enableBody(bodyId: BodyId): void {
        this.checkDestroyed();
        this.wasm._b2Glue_EnableBody(bodyId);
    }

    public disableBody(bodyId: BodyId): void {
        this.checkDestroyed();
        this.wasm._b2Glue_DisableBody(bodyId);
    }

    public isBodyEnabled(bodyId: BodyId): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_BodyIsEnabled(bodyId) !== 0;
    }

    /** impulses holds x, y per body. Applied at each body's center of mass, in one call. */
    public applyLinearImpulseBatch(bodyIds: ArrayLike<BodyId>, impulses: ArrayLike<number>, wake: boolean = true): void {
        this.checkDestroyed();
        const count = bodyIds.length;
        if (count === 0) return;
        if (impulses.length < count * 2) {
            throw new Error(`Box2DWorld.applyLinearImpulseBatch: need 2 values per body (got ${impulses.length} for ${count} bodies).`);
        }
        const idsBytes = count * 8;
        this.withBytes(idsBytes + count * 8, p => {
            this.writeIds(p, bodyIds);
            this.writeFloats(p + idsBytes, impulses, count * 2);
            this.wasm._b2Glue_ApplyLinearImpulseBatch(p, count, p + idsBytes, wake);
        });
    }

    /** angular holds one value per body: an impulse about the center of mass. */
    public applyAngularImpulseBatch(bodyIds: ArrayLike<BodyId>, angular: ArrayLike<number>, wake: boolean = true): void {
        this.checkDestroyed();
        const count = bodyIds.length;
        if (count === 0) return;
        if (angular.length < count) {
            throw new Error(`Box2DWorld.applyAngularImpulseBatch: need 1 value per body (got ${angular.length} for ${count} bodies).`);
        }
        const idsBytes = count * 8;
        this.withBytes(idsBytes + count * 4, p => {
            this.writeIds(p, bodyIds);
            this.writeFloats(p + idsBytes, angular, count);
            this.wasm._b2Glue_ApplyAngularImpulseBatch(p, count, p + idsBytes, wake);
        });
    }

    // ---- Joints: wheel, mouse, filter; limits, springs, motors by kind; reaction ----

    /** Body B slides and rotates along the axis of frame A. Spring, limit and motor are optional. */
    public createWheelJoint(options: WheelJointOptions): JointId {
        this.checkDestroyed();
        const a = options.anchorA ?? [0, 0];
        const b = options.anchorB ?? [0, 0];
        const axis = options.axis ?? [1, 0];
        const spring = options.spring;
        const lim = options.limits;
        const motor = options.motor;
        const p = this.scratchPtr;
        this.writeFloats(p, [a[0], a[1], b[0], b[1], axis[0], axis[1]]);
        return this.wasm._b2Glue_CreateWheel(
            this.worldId,
            options.bodyA,
            options.bodyB,
            p,
            p + 8,
            p + 16,
            spring ? 1 : 0,
            spring?.hertz ?? 0.0,
            spring?.dampingRatio ?? 0.0,
            lim ? 1 : 0,
            lim ? lim[0] : 0.0,
            lim ? lim[1] : 0.0,
            motor ? 1 : 0,
            motor?.speed ?? 0.0,
            motor?.maxTorque ?? 0.0
        );
    }

    /**
     * Mouse joint: a motor joint with a linear spring and no velocity motor (Box2D 3.2 has no mouse joint).
     * Pass the ground body as bodyA. Defaults: maxForce 1000, hertz 5, dampingRatio 0.7.
     */
    public createMouseJoint(options: MouseJointOptions): JointId {
        this.checkDestroyed();
        const anchor = options.anchorB ?? [0, 0];
        const p = this.scratchPtr;
        this.writeFloats(p, [options.target[0], options.target[1], anchor[0], anchor[1]]);
        return this.wasm._b2Glue_CreateMouse(
            this.worldId,
            options.bodyA,
            options.bodyB,
            p,
            p + 8,
            options.maxForce ?? 1000.0,
            options.hertz ?? 5.0,
            options.dampingRatio ?? 0.7
        );
    }

    /** Moves the mouse joint's target (world). Returns false when the joint is not a mouse joint. */
    public setMouseJointTarget(jointId: JointId, target: Vec2Tuple): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_MouseSetTarget(jointId, target[0], target[1]) !== 0;
    }

    /** Disables collision between two bodies and keeps them in one island. Has no other effect. */
    public createFilterJoint(options: FilterJointOptions): JointId {
        this.checkDestroyed();
        return this.wasm._b2Glue_CreateFilter(this.worldId, options.bodyA, options.bodyB);
    }

    /** b2JointType value. Compare with JointType.Wheel and the other constants. */
    public getJointType(jointId: JointId): JointTypeValue {
        this.checkDestroyed();
        return this.wasm._b2Glue_JointType(jointId) as JointTypeValue;
    }

    /** Limit range: angle for revolute, translation for prismatic and wheel, length for distance. Returns false for other kinds. */
    public setJointLimits(jointId: JointId, limits: [number, number]): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_JointSetLimits(jointId, limits[0], limits[1]) !== 0;
    }

    public enableJointLimit(jointId: JointId, enable: boolean): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_JointEnableLimit(jointId, enable) !== 0;
    }

    /** Spring for distance, prismatic, revolute and wheel joints. Returns false for other kinds. */
    public enableJointSpring(jointId: JointId, enable: boolean): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_JointEnableSpring(jointId, enable) !== 0;
    }

    public setJointSpring(jointId: JointId, hertz: number, dampingRatio: number): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_JointSetSpring(jointId, hertz, dampingRatio) !== 0;
    }

    /** Motor for distance, prismatic, revolute and wheel joints. Returns false for other kinds. */
    public enableJointMotor(jointId: JointId, enable: boolean): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_JointEnableMotor(jointId, enable) !== 0;
    }

    /** speed is rad/s or m/s. maxForce is a force for prismatic and distance joints, a torque for revolute and wheel. */
    public setJointMotor(jointId: JointId, speed: number, maxForce: number): boolean {
        this.checkDestroyed();
        return this.wasm._b2Glue_JointSetMotor(jointId, speed, maxForce) !== 0;
    }

    /** Force and torque on body B from the last step. Null for an invalid joint. */
    public getJointReaction(jointId: JointId): JointReaction | null {
        this.checkDestroyed();
        const p = this.scratchPtr;
        if (this.wasm._b2Glue_JointReaction(jointId, p) === 0) return null;
        const f = this.readFloats(p, 3);
        return { force: [f[0], f[1]], torque: f[2] };
    }

    // ---- Queries: shape cast, overlaps, all-hits ray cast, explosion ----

    /**
     * Closest hit of a point-cloud proxy (1 to 8 points plus a radius) moved from origin along translation.
     * Null on a miss. Throws RangeError for a proxy outside 1..8 points.
     */
    public castShapeClosest(points: Vec2Tuple[], radius: number, origin: Vec2Tuple, translation: Vec2Tuple): QueryHit | null {
        this.checkDestroyed();
        const count = this.checkProxy(points);
        const p = this.scratchPtr; // points (up to 64 bytes), origin, translation, one hit: 112 bytes of the 128-byte scratch
        this.writeFloats(p, points.flat());
        this.writeFloats(p + 64, [origin[0], origin[1], translation[0], translation[1]]);
        if (this.wasm._b2Glue_CastShapeClosest(this.worldId, p, count, radius, p + 64, p + 72, p + 80) !== 1) return null;
        return this.readHit(p + 80);
    }

    /** Shapes whose bounds overlap the box. total can exceed shapes.length when maxResults is smaller. */
    public overlapAABB(min: Vec2Tuple, max: Vec2Tuple, maxResults: number = 64): OverlapResult {
        this.checkDestroyed();
        return this.collectOverlap(maxResults, (out, cap) =>
            this.wasm._b2Glue_OverlapAABB(this.worldId, min[0], min[1], max[0], max[1], out, cap)
        );
    }

    public overlapCircle(center: Vec2Tuple, radius: number, maxResults: number = 64): OverlapResult {
        this.checkDestroyed();
        return this.collectOverlap(maxResults, (out, cap) =>
            this.wasm._b2Glue_OverlapCircle(this.worldId, center[0], center[1], radius, out, cap)
        );
    }

    /** Shapes overlapping a convex proxy of 1 to 8 points plus a radius. Throws RangeError outside that range. */
    public overlapPolygon(points: Vec2Tuple[], radius: number, maxResults: number = 64): OverlapResult {
        this.checkDestroyed();
        const count = this.checkProxy(points);
        const flat = points.flat();
        return this.withBytes(count * 8, p => {
            this.writeFloats(p, flat);
            return this.collectOverlap(maxResults, (out, cap) =>
                this.wasm._b2Glue_OverlapPolygon(this.worldId, p, count, radius, out, cap)
            );
        });
    }

    /** Every hit of a ray from origin along translation, in no particular order. */
    public castRayAll(origin: Vec2Tuple, translation: Vec2Tuple, maxHits: number = 64): RayAllResult {
        this.checkDestroyed();
        const cap = Math.max(0, Math.floor(maxHits));
        return this.withBytes(16 + Math.max(cap, 1) * HIT_BYTES, buf => {
            this.writeFloats(buf, [origin[0], origin[1], translation[0], translation[1]]);
            const total = this.wasm._b2Glue_CastRayAll(this.worldId, buf, buf + 8, buf + 16, cap);
            const n = Math.min(total, cap);
            const hits: QueryHit[] = [];
            for (let i = 0; i < n; i++) hits.push(this.readHit(buf + 16 + i * HIT_BYTES));
            return { total, hits };
        });
    }

    /**
     * Radial impulse on every awake dynamic shape within radius + falloff. Full strength inside radius, linear falloff
     * to zero at radius + falloff. maskBits selects the shapes it can reach (default: all).
     */
    public explode(center: Vec2Tuple, radius: number, falloff: number, impulsePerLength: number, maskBits: bigint = ALL_BITS): void {
        this.checkDestroyed();
        this.wasm._b2Glue_Explode(this.worldId, center[0], center[1], radius, falloff, impulsePerLength, maskBits);
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

    private checkProxy(points: Vec2Tuple[]): number {
        if (points.length < 1 || points.length > MAX_PROXY_POINTS) {
            throw new RangeError(`Box2DWorld: a proxy needs 1 to ${MAX_PROXY_POINTS} points (got ${points.length}).`);
        }
        return points.length;
    }

    // Temporary heap block for calls that take pointers to several values. The caller's callback must not keep the pointer.
    private withBytes<T>(bytes: number, fn: (ptr: number) => T): T {
        const ptr = this.wasm._malloc(bytes);
        try {
            return fn(ptr);
        } finally {
            this.wasm._free(ptr);
        }
    }

    private writeFloats(ptr: number, values: ArrayLike<number>, count: number = values.length): void {
        const f32 = this.getFloat32View();
        const offset = ptr >> 2;
        for (let i = 0; i < count; i++) f32[offset + i] = values[i];
    }

    private readFloats(ptr: number, count: number): number[] {
        const offset = ptr >> 2;
        return Array.from(this.getFloat32View().subarray(offset, offset + count));
    }

    private writeIds(ptr: number, ids: ArrayLike<bigint>): void {
        const u64 = this.getBigUint64View();
        const offset = ptr >> 3;
        for (let i = 0; i < ids.length; i++) u64[offset + i] = ids[i];
    }

    private readHit(ptr: number): QueryHit {
        const u64 = this.getBigUint64View();
        const f32 = this.getFloat32View();
        const f = ptr >> 2;
        return {
            shape: u64[ptr >> 3],
            fraction: f32[f + 2],
            point: [f32[f + 3], f32[f + 4]],
            normal: [f32[f + 5], f32[f + 6]]
        };
    }

    // Shared by the overlap queries. call writes at most cap ids to out and returns the total count.
    private collectOverlap(maxResults: number, call: (out: number, cap: number) => number): OverlapResult {
        const cap = Math.max(0, Math.floor(maxResults));
        return this.withBytes(Math.max(cap, 1) * 8, out => {
            const total = call(out, cap);
            const n = Math.min(total, cap);
            const u64 = this.getBigUint64View();
            const shapes: ShapeId[] = [];
            for (let i = 0; i < n; i++) shapes.push(u64[(out >> 3) + i]);
            return { total, shapes };
        });
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
