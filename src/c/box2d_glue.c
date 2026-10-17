// SPDX-License-Identifier: MIT
// Box2D WebAssembly High-Performance Glue Implementation

#include "box2d_glue.h"
#include "physics_world.h"
#include <string.h>
#include <math.h>

static inline b2WorldId worldOf(uint32_t val) {
    return b2LoadWorldId(val);
}

static inline b2BodyId bodyOf(uint64_t val) {
    return b2LoadBodyId(val);
}

static inline b2ShapeId shapeOf(uint64_t val) {
    return b2LoadShapeId(val);
}

static inline b2JointId jointOf(uint64_t val) {
    return b2LoadJointId(val);
}

// ---- World Management ----

uint32_t b2Glue_CreateWorld(float gx, float gy, int enableSleep, int enableContinuous) {
    b2WorldDef def = b2DefaultWorldDef();
    def.gravity = (b2Vec2){ gx, gy };
    def.enableSleep = enableSleep != 0;
    def.enableContinuous = enableContinuous != 0;
    return b2StoreWorldId(b2CreateWorld(&def));
}

void b2Glue_DestroyWorld(uint32_t worldIdRaw) {
    b2DestroyWorld(worldOf(worldIdRaw));
}

void b2Glue_Step(uint32_t worldIdRaw, float timeStep, int subStepCount) {
    b2World_Step(worldOf(worldIdRaw), timeStep, subStepCount);
}

bool b2Glue_WorldIsValid(uint32_t worldIdRaw) {
    return b2World_IsValid(worldOf(worldIdRaw));
}

void b2Glue_SetGravity(uint32_t worldIdRaw, float gx, float gy) {
    b2World_SetGravity(worldOf(worldIdRaw), (b2Vec2){ gx, gy });
}

void b2Glue_Configure(uint32_t worldIdRaw, bool sleeping, bool continuous) {
    b2World_EnableSleeping(worldOf(worldIdRaw), sleeping);
    b2World_EnableContinuous(worldOf(worldIdRaw), continuous);
}

void b2Glue_SetContactTuning(uint32_t worldIdRaw, float hertz, float dampingRatio, float pushSpeed) {
    b2World_SetContactTuning(worldOf(worldIdRaw), hertz, dampingRatio, pushSpeed);
}

uint64_t b2Glue_StateHash(uint32_t worldIdRaw) {
    return b2World_GetStateHash(worldOf(worldIdRaw));
}

void b2Glue_SetTaskSystem(uint32_t worldIdRaw, int workers, b2EnqueueTaskCallback* enqueue, b2FinishTaskCallback* finish) {
    b2WorldId id = worldOf(worldIdRaw);
    b2World* w = b2GetWorldFromId(id);
    if (w == NULL || workers < 2 || enqueue == NULL || finish == NULL) return;
    w->enqueueTaskFcn = enqueue;
    w->finishTaskFcn = finish;
    w->userTaskContext = NULL;
    b2World_SetWorkerCount(id, workers);
}

int b2Glue_WorkerCount(uint32_t worldIdRaw) {
    return b2World_GetWorkerCount(worldOf(worldIdRaw));
}

// ---- Body Management ----

uint64_t b2Glue_CreateBody(uint32_t worldIdRaw, int type, float px, float py, float rotation) {
    b2BodyDef def = b2DefaultBodyDef();
    def.type = (b2BodyType)type;
    def.position = (b2Vec2){ px, py };
    def.rotation = b2MakeRot(rotation);
    return b2StoreBodyId(b2CreateBody(worldOf(worldIdRaw), &def));
}

// flags: bit 0: enableSleep, bit 1: isBullet, bit 2: fixedRotation, bit 3: startAsleep
uint64_t b2Glue_CreateBodyEx(uint32_t worldIdRaw, int type, const float* pose3, const float* vel3, int flags) {
    b2BodyDef def = b2DefaultBodyDef();
    def.type = (b2BodyType)type;
    def.position = (b2Vec2){ pose3[0], pose3[1] };
    def.rotation = b2MakeRot(pose3[2]);
    def.linearVelocity = (b2Vec2){ vel3[0], vel3[1] };
    def.angularVelocity = vel3[2];
    def.enableSleep = (flags & 1) != 0;
    def.isBullet = (flags & 2) != 0;
    def.motionLocks.angularZ = (flags & 4) != 0;
    def.isAwake = (flags & 8) == 0;
    return b2StoreBodyId(b2CreateBody(worldOf(worldIdRaw), &def));
}

void b2Glue_DestroyBody(uint64_t bodyIdRaw) {
    b2DestroyBody(bodyOf(bodyIdRaw));
}

bool b2Glue_BodyIsValid(uint64_t bodyIdRaw) {
    return b2Body_IsValid(bodyOf(bodyIdRaw));
}

void b2Glue_SetTransform(uint64_t bodyIdRaw, float px, float py, float rotation) {
    b2Body_SetTransform(bodyOf(bodyIdRaw), (b2Vec2){ px, py }, b2MakeRot(rotation));
}

void b2Glue_GetTransform(uint64_t bodyIdRaw, float* out3) {
    b2WorldTransform t = b2Body_GetTransform(bodyOf(bodyIdRaw));
    out3[0] = t.p.x;
    out3[1] = t.p.y;
    out3[2] = b2Rot_GetAngle(t.q);
}

void b2Glue_SetTargetTransform(uint64_t bodyIdRaw, const float* pose3, float dt) {
    b2WorldTransform t;
    t.p = (b2Vec2){ pose3[0], pose3[1] };
    t.q = b2MakeRot(pose3[2]);
    b2Body_SetTargetTransform(bodyOf(bodyIdRaw), t, dt, true);
}

void b2Glue_SetLinearVelocity(uint64_t bodyIdRaw, float vx, float vy) {
    b2Body_SetLinearVelocity(bodyOf(bodyIdRaw), (b2Vec2){ vx, vy });
}

void b2Glue_GetLinearVelocity(uint64_t bodyIdRaw, float* out2) {
    b2Vec2 v = b2Body_GetLinearVelocity(bodyOf(bodyIdRaw));
    out2[0] = v.x;
    out2[1] = v.y;
}

void b2Glue_SetAngularVelocity(uint64_t bodyIdRaw, float omega) {
    b2Body_SetAngularVelocity(bodyOf(bodyIdRaw), omega);
}

float b2Glue_GetAngularVelocity(uint64_t bodyIdRaw) {
    return b2Body_GetAngularVelocity(bodyOf(bodyIdRaw));
}

void b2Glue_ApplyForce(uint64_t bodyIdRaw, float fx, float fy, float px, float py, bool wake) {
    b2Body_ApplyForce(bodyOf(bodyIdRaw), (b2Vec2){ fx, fy }, (b2Vec2){ px, py }, wake);
}

void b2Glue_ApplyForceToCenter(uint64_t bodyIdRaw, float fx, float fy, bool wake) {
    b2Body_ApplyForceToCenter(bodyOf(bodyIdRaw), (b2Vec2){ fx, fy }, wake);
}

void b2Glue_ApplyLinearImpulse(uint64_t bodyIdRaw, float ix, float iy, float px, float py, bool wake) {
    b2Body_ApplyLinearImpulse(bodyOf(bodyIdRaw), (b2Vec2){ ix, iy }, (b2Vec2){ px, py }, wake);
}

void b2Glue_ApplyLinearImpulseToCenter(uint64_t bodyIdRaw, float ix, float iy, bool wake) {
    b2Body_ApplyLinearImpulseToCenter(bodyOf(bodyIdRaw), (b2Vec2){ ix, iy }, wake);
}

void b2Glue_ApplyAngularImpulse(uint64_t bodyIdRaw, float impulse, bool wake) {
    b2Body_ApplyAngularImpulse(bodyOf(bodyIdRaw), impulse, wake);
}

float b2Glue_BodyMass(uint64_t bodyIdRaw) {
    return b2Body_GetMass(bodyOf(bodyIdRaw));
}

void b2Glue_SetBodyMass(uint64_t bodyIdRaw, float mass) {
    b2BodyId id = bodyOf(bodyIdRaw);
    b2MassData m = b2Body_GetMassData(id);
    if (m.mass <= 0.0f) return;
    float k = mass / m.mass;
    m.mass = mass;
    m.rotationalInertia *= k;
    b2Body_SetMassData(id, m);
}

bool b2Glue_BodyIsAwake(uint64_t bodyIdRaw) {
    return b2Body_IsAwake(bodyOf(bodyIdRaw));
}

void b2Glue_SetSleepThreshold(uint64_t bodyIdRaw, float threshold) {
    b2Body_SetSleepThreshold(bodyOf(bodyIdRaw), threshold);
}

// ---- Batch and Event State ----

void b2Glue_GetBatchTransforms(const uint64_t* ids, int count, float* out3) {
    for (int i = 0; i < count; ++i) {
        b2WorldTransform t = b2Body_GetTransform(bodyOf(ids[i]));
        float* o = out3 + i * 3;
        o[0] = t.p.x;
        o[1] = t.p.y;
        o[2] = b2Rot_GetAngle(t.q);
    }
}

// 6 floats per body: px, py, rotation, vx, vy, angularVelocity
void b2Glue_GetBatchState(const uint64_t* ids, int count, float* out6) {
    for (int i = 0; i < count; ++i) {
        b2BodyId id = bodyOf(ids[i]);
        b2WorldTransform t = b2Body_GetTransform(id);
        b2Vec2 v = b2Body_GetLinearVelocity(id);
        float w = b2Body_GetAngularVelocity(id);
        float* o = out6 + i * 6;
        o[0] = t.p.x;
        o[1] = t.p.y;
        o[2] = b2Rot_GetAngle(t.q);
        o[3] = v.x;
        o[4] = v.y;
        o[5] = w;
    }
}

int b2Glue_MovedBodies(uint32_t worldIdRaw, uint64_t* ids, float* out6, int max) {
    b2BodyEvents ev = b2World_GetBodyEvents(worldOf(worldIdRaw));
    int n = ev.moveCount < max ? ev.moveCount : max;
    for (int i = 0; i < n; ++i) {
        const b2BodyMoveEvent* e = ev.moveEvents + i;
        ids[i] = b2StoreBodyId(e->bodyId);
        b2Vec2 v = b2Body_GetLinearVelocity(e->bodyId);
        float w = b2Body_GetAngularVelocity(e->bodyId);
        float* o = out6 + i * 6;
        o[0] = e->transform.p.x;
        o[1] = e->transform.p.y;
        o[2] = b2Rot_GetAngle(e->transform.q);
        o[3] = v.x;
        o[4] = v.y;
        o[5] = w;
    }
    return ev.moveCount;
}

// ---- Shapes ----

static b2ShapeDef makeShapeDef(float density, float friction, float restitution, int flags) {
    b2ShapeDef def = b2DefaultShapeDef();
    def.density = density;
    def.material.friction = friction;
    def.material.restitution = restitution;
    def.isSensor = (flags & 1) != 0;
    def.enableSensorEvents = true;
    def.enableContactEvents = (flags & 2) != 0;
    def.enableHitEvents = (flags & 4) != 0;
    return def;
}

uint64_t b2Glue_AddCircle(uint64_t bodyIdRaw, float radius, float density, float friction, float restitution, int flags) {
    b2ShapeDef def = makeShapeDef(density, friction, restitution, flags);
    b2Circle circle = { .center = { 0.0f, 0.0f }, .radius = radius };
    return b2StoreShapeId(b2CreateCircleShape(bodyOf(bodyIdRaw), &def, &circle));
}

uint64_t b2Glue_AddBox(uint64_t bodyIdRaw, float hx, float hy, float density, float friction, float restitution, int flags) {
    b2ShapeDef def = makeShapeDef(density, friction, restitution, flags);
    b2Polygon box = b2MakeBox(hx, hy);
    return b2StoreShapeId(b2CreatePolygonShape(bodyOf(bodyIdRaw), &def, &box));
}

uint64_t b2Glue_AddCapsule(uint64_t bodyIdRaw, float halfHeight, float radius, float density, float friction, float restitution, int flags) {
    b2ShapeDef def = makeShapeDef(density, friction, restitution, flags);
    b2Capsule cap = { .center1 = { 0.0f, -halfHeight }, .center2 = { 0.0f, halfHeight }, .radius = radius };
    return b2StoreShapeId(b2CreateCapsuleShape(bodyOf(bodyIdRaw), &def, &cap));
}

uint64_t b2Glue_AddSegment(uint64_t bodyIdRaw, float x1, float y1, float x2, float y2, float density, float friction, float restitution, int flags) {
    b2ShapeDef def = makeShapeDef(density, friction, restitution, flags);
    b2Segment seg = { .point1 = { x1, y1 }, .point2 = { x2, y2 } };
    return b2StoreShapeId(b2CreateSegmentShape(bodyOf(bodyIdRaw), &def, &seg));
}

uint64_t b2Glue_AddPolygon(uint64_t bodyIdRaw, const float* points2, int count, float radius, float density, float friction, float restitution, int flags) {
    if (count < 3 || points2 == NULL) return 0;
    b2ShapeDef def = makeShapeDef(density, friction, restitution, flags);
    b2Hull hull = b2ComputeHull((const b2Vec2*)points2, count);
    if (hull.count < 3) return 0;
    b2Polygon poly = b2MakePolygon(&hull, radius);
    return b2StoreShapeId(b2CreatePolygonShape(bodyOf(bodyIdRaw), &def, &poly));
}

void b2Glue_DestroyShape(uint64_t shapeIdRaw, bool updateBodyMass) {
    b2DestroyShape(shapeOf(shapeIdRaw), updateBodyMass);
}

bool b2Glue_ShapeIsValid(uint64_t shapeIdRaw) {
    return b2Shape_IsValid(shapeOf(shapeIdRaw));
}

uint64_t b2Glue_ShapeBody(uint64_t shapeIdRaw) {
    return b2StoreBodyId(b2Shape_GetBody(shapeOf(shapeIdRaw)));
}

// ---- Joints ----

uint64_t b2Glue_CreateWeld(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, float linearHertz, float angularHertz) {
    b2WeldJointDef def = b2DefaultWeldJointDef();
    def.base.bodyIdA = bodyOf(a);
    def.base.bodyIdB = bodyOf(b);
    def.base.localFrameA.p = (b2Vec2){ anchorA2[0], anchorA2[1] };
    def.base.localFrameB.p = (b2Vec2){ anchorB2[0], anchorB2[1] };
    def.base.collideConnected = false;
    def.linearHertz = linearHertz;
    def.angularHertz = angularHertz;
    return b2StoreJointId(b2CreateWeldJoint(worldOf(worldIdRaw), &def));
}

uint64_t b2Glue_CreateRevolute(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, int limited, float lower, float upper) {
    b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
    def.base.bodyIdA = bodyOf(a);
    def.base.bodyIdB = bodyOf(b);
    def.base.localFrameA.p = (b2Vec2){ anchorA2[0], anchorA2[1] };
    def.base.localFrameB.p = (b2Vec2){ anchorB2[0], anchorB2[1] };
    def.base.collideConnected = false;
    if (limited) {
        def.enableLimit = true;
        def.lowerAngle = lower;
        def.upperAngle = upper;
    }
    return b2StoreJointId(b2CreateRevoluteJoint(worldOf(worldIdRaw), &def));
}

uint64_t b2Glue_CreatePrismatic(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, const float* axis2, int limited, float lower, float upper) {
    b2PrismaticJointDef def = b2DefaultPrismaticJointDef();
    def.base.bodyIdA = bodyOf(a);
    def.base.bodyIdB = bodyOf(b);
    def.base.localFrameA.p = (b2Vec2){ anchorA2[0], anchorA2[1] };
    def.base.localFrameB.p = (b2Vec2){ anchorB2[0], anchorB2[1] };
    def.base.collideConnected = false;
    float angle = atan2f(axis2[1], axis2[0]);
    def.base.localFrameA.q = b2MakeRot(angle);
    def.base.localFrameB.q = b2MakeRot(angle);
    if (limited) {
        def.enableLimit = true;
        def.lowerTranslation = lower;
        def.upperTranslation = upper;
    }
    return b2StoreJointId(b2CreatePrismaticJoint(worldOf(worldIdRaw), &def));
}

uint64_t b2Glue_CreateDistance(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, float length, float hertz, float dampingRatio) {
    b2DistanceJointDef def = b2DefaultDistanceJointDef();
    def.base.bodyIdA = bodyOf(a);
    def.base.bodyIdB = bodyOf(b);
    def.base.localFrameA.p = (b2Vec2){ anchorA2[0], anchorA2[1] };
    def.base.localFrameB.p = (b2Vec2){ anchorB2[0], anchorB2[1] };
    def.base.collideConnected = false;
    def.length = length;
    def.enableSpring = hertz > 0.0f;
    def.hertz = hertz;
    def.dampingRatio = dampingRatio;
    return b2StoreJointId(b2CreateDistanceJoint(worldOf(worldIdRaw), &def));
}

uint64_t b2Glue_CreateMotor(uint32_t worldIdRaw, uint64_t a, uint64_t b, float maxForce, float maxTorque) {
    b2MotorJointDef def = b2DefaultMotorJointDef();
    def.base.bodyIdA = bodyOf(a);
    def.base.bodyIdB = bodyOf(b);
    def.base.collideConnected = false;
    def.maxVelocityForce = maxForce;
    def.maxVelocityTorque = maxTorque;
    return b2StoreJointId(b2CreateMotorJoint(worldOf(worldIdRaw), &def));
}

void b2Glue_DestroyJoint(uint64_t jointIdRaw) {
    b2DestroyJoint(jointOf(jointIdRaw));
}

bool b2Glue_JointIsValid(uint64_t jointIdRaw) {
    return b2Joint_IsValid(jointOf(jointIdRaw));
}

// ---- Events ----

int b2Glue_ContactBeginEvents(uint32_t worldIdRaw, uint64_t* out2, int max) {
    b2ContactEvents ev = b2World_GetContactEvents(worldOf(worldIdRaw));
    int n = ev.beginCount < max ? ev.beginCount : max;
    for (int i = 0; i < n; ++i) {
        out2[2 * i] = b2StoreShapeId(ev.beginEvents[i].shapeIdA);
        out2[2 * i + 1] = b2StoreShapeId(ev.beginEvents[i].shapeIdB);
    }
    return n;
}

int b2Glue_ContactEndEvents(uint32_t worldIdRaw, uint64_t* out2, int max) {
    b2ContactEvents ev = b2World_GetContactEvents(worldOf(worldIdRaw));
    int n = ev.endCount < max ? ev.endCount : max;
    for (int i = 0; i < n; ++i) {
        out2[2 * i] = b2StoreShapeId(ev.endEvents[i].shapeIdA);
        out2[2 * i + 1] = b2StoreShapeId(ev.endEvents[i].shapeIdB);
    }
    return n;
}

int b2Glue_SensorBeginEvents(uint32_t worldIdRaw, uint64_t* out2, int max) {
    b2SensorEvents ev = b2World_GetSensorEvents(worldOf(worldIdRaw));
    int n = ev.beginCount < max ? ev.beginCount : max;
    for (int i = 0; i < n; ++i) {
        out2[2 * i] = b2StoreShapeId(ev.beginEvents[i].sensorShapeId);
        out2[2 * i + 1] = b2StoreShapeId(ev.beginEvents[i].visitorShapeId);
    }
    return n;
}

int b2Glue_SensorEndEvents(uint32_t worldIdRaw, uint64_t* out2, int max) {
    b2SensorEvents ev = b2World_GetSensorEvents(worldOf(worldIdRaw));
    int n = ev.endCount < max ? ev.endCount : max;
    for (int i = 0; i < n; ++i) {
        out2[2 * i] = b2StoreShapeId(ev.endEvents[i].sensorShapeId);
        out2[2 * i + 1] = b2StoreShapeId(ev.endEvents[i].visitorShapeId);
    }
    return n;
}

// ---- Queries & Raycast ----

uint64_t b2Glue_CastRayClosest(uint32_t worldIdRaw, const float* origin2, const float* translation2, float* outResult6) {
    b2RayResult r = b2World_CastRayClosest(worldOf(worldIdRaw), (b2Vec2){ origin2[0], origin2[1] }, (b2Vec2){ translation2[0], translation2[1] }, b2DefaultQueryFilter());
    if (!r.hit) return 0;
    if (outResult6) {
        outResult6[0] = r.fraction;
        outResult6[1] = r.point.x;
        outResult6[2] = r.point.y;
        outResult6[3] = r.normal.x;
        outResult6[4] = r.normal.y;
        outResult6[5] = 1.0f;
    }
    return b2StoreShapeId(r.shapeId);
}

// ---- Character Mover ----

#define GLUE_MAX_PLANES 32

typedef struct {
    b2ShapeId self;
    b2CollisionPlane planes[GLUE_MAX_PLANES];
    int count;
} MoverContext;

static bool collectPlanes(b2ShapeId shapeId, const b2PlaneResult* result, void* ctx) {
    MoverContext* c = (MoverContext*)ctx;
    if (B2_ID_EQUALS(shapeId, c->self)) return true;
    if (!result->hit) return true;
    if (c->count < GLUE_MAX_PLANES) {
        c->planes[c->count++] = (b2CollisionPlane){ .plane = result->plane, .pushLimit = 1e30f, .push = 0.0f, .clipVelocity = true };
    }
    return true;
}

static void gatherPlanes(b2WorldId w, MoverContext* c, const float* pos, const b2Capsule* cap) {
    c->count = 0;
    b2World_CollideMover(w, (b2Vec2){ pos[0], pos[1] }, cap, b2DefaultQueryFilter(), collectPlanes, c);
}

int b2Glue_MoveCharacter(uint32_t worldIdRaw, uint64_t selfShapeIdRaw, float* pos2, float halfHeight, float radius, const float* delta2, float groundNormalY) {
    b2WorldId w = worldOf(worldIdRaw);
    b2Capsule cap = { .center1 = { 0.0f, -halfHeight }, .center2 = { 0.0f, halfHeight }, .radius = radius };
    b2ShapeId selfShape = shapeOf(selfShapeIdRaw);
    b2Filter savedFilter = { 0 };
    bool hasSelf = B2_IS_NON_NULL(selfShape) && b2Shape_IsValid(selfShape);
    if (hasSelf) {
        savedFilter = b2Shape_GetFilter(selfShape);
        b2Filter noCollide = { 0 };
        b2Shape_SetFilter(selfShape, noCollide);
    }

    MoverContext c;
    c.self = selfShape;
    float target[2] = { pos2[0] + delta2[0], pos2[1] + delta2[1] };

    // Settle first only when not rising (jumping/climbing)
    if (delta2[1] <= 0.0f) {
        gatherPlanes(w, &c, pos2, &cap);
        if (c.count > 0) {
            b2PlaneSolverResult s = b2SolvePlanes((b2Vec2){ 0.0f, 0.0f }, c.planes, c.count);
            pos2[0] += s.delta.x;
            pos2[1] += s.delta.y;
        }
    }

    for (int i = 0; i < 5; ++i) {
        gatherPlanes(w, &c, pos2, &cap);
        b2Vec2 remaining = { target[0] - pos2[0], target[1] - pos2[1] };
        b2Vec2 step = remaining;
        if (c.count > 0) {
            step = b2SolvePlanes(remaining, c.planes, c.count).delta;
        }
        if (b2LengthSquared(step) < 1e-10f) break;
        float fraction = b2World_CastMover(w, (b2Vec2){ pos2[0], pos2[1] }, &cap, step, b2DefaultQueryFilter());
        pos2[0] += step.x * fraction;
        pos2[1] += step.y * fraction;
        if (fraction >= 1.0f) break;
    }

    int grounded = 0;
    gatherPlanes(w, &c, pos2, &cap);
    for (int i = 0; i < c.count; ++i) {
        if (c.planes[i].plane.normal.y >= groundNormalY) {
            grounded = 1;
            break;
        }
    }

    // If blocked horizontally and walking, attempt autostep up to 0.3f
    float remX = target[0] - pos2[0];
    if (fabsf(remX) > 1e-4f && fabsf(delta2[0]) > 1e-4f) {
        float maxStep = 0.3f;
        float upFrac = b2World_CastMover(w, (b2Vec2){ pos2[0], pos2[1] }, &cap, (b2Vec2){ 0.0f, maxStep }, b2DefaultQueryFilter());
        float liftY = pos2[1] + maxStep * upFrac;
        if (liftY > pos2[1] + 0.01f) {
            float hFrac = b2World_CastMover(w, (b2Vec2){ pos2[0], liftY }, &cap, (b2Vec2){ remX, 0.0f }, b2DefaultQueryFilter());
            if (hFrac > 0.01f) {
                float stepX = pos2[0] + remX * hFrac;
                float downDist = (liftY - pos2[1]) + 0.05f;
                float downFrac = b2World_CastMover(w, (b2Vec2){ stepX, liftY }, &cap, (b2Vec2){ 0.0f, -downDist }, b2DefaultQueryFilter());
                float stepY = liftY - downDist * downFrac;
                // Verify landing is on ground
                gatherPlanes(w, &c, (float[]){ stepX, stepY }, &cap);
                for (int i = 0; i < c.count; ++i) {
                    if (c.planes[i].plane.normal.y >= groundNormalY) {
                        pos2[0] = stepX;
                        pos2[1] = stepY;
                        if (c.count > 0) {
                            b2PlaneSolverResult s = b2SolvePlanes((b2Vec2){ 0.0f, 0.0f }, c.planes, c.count);
                            pos2[0] += s.delta.x;
                            pos2[1] += s.delta.y;
                        }
                        grounded = 1;
                        break;
                    }
                }
            }
        }
    }

    if (hasSelf) {
        b2Shape_SetFilter(selfShape, savedFilter);
    }
    return grounded;
}

// ---- Snapshots ----

int b2Glue_SnapshotSize(uint32_t worldIdRaw) {
    return b2World_GetSnapshot(worldOf(worldIdRaw), NULL, 0);
}

int b2Glue_SnapshotTake(uint32_t worldIdRaw, uint8_t* outBuffer, int capacity) {
    return b2World_GetSnapshot(worldOf(worldIdRaw), outBuffer, capacity);
}

bool b2Glue_SnapshotRestore(uint32_t worldIdRaw, const uint8_t* buffer, int size) {
    return b2World_Restore(worldOf(worldIdRaw), buffer, size);
}

uint32_t b2Glue_SnapshotCreateWorld(const uint8_t* buffer, int size, int workerCount) {
    return b2StoreWorldId(b2CreateWorldFromSnapshot(buffer, size, workerCount));
}
