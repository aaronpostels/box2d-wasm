// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: joints. Wheel, mouse and filter creation, limits, springs and motors per kind, reaction forces.
// Every function that takes a joint id checks b2Joint_IsValid first: a stale id returns the failure value instead of
// hitting a Box2D assert.
#include "box2d_glue_joints.h"
#include <math.h>

static inline b2WorldId jWorld(uint32_t val) { return b2LoadWorldId(val); }
static inline b2BodyId jBody(uint64_t val) { return b2LoadBodyId(val); }
static inline b2JointId jJoint(uint64_t val) { return b2LoadJointId(val); }

static void setFramesAB(b2JointDef* base, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2) {
    base->bodyIdA = jBody(a);
    base->bodyIdB = jBody(b);
    base->localFrameA.p = (b2Vec2){ anchorA2[0], anchorA2[1] };
    base->localFrameB.p = (b2Vec2){ anchorB2[0], anchorB2[1] };
    base->collideConnected = false;
}

// ---- Creation ----

uint64_t b2Glue_CreateWheel(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, const float* axis2, int enableSpring, float hertz, float dampingRatio, int enableLimit, float lower, float upper, int enableMotor, float motorSpeed, float maxMotorTorque) {
    b2WheelJointDef def = b2DefaultWheelJointDef();
    setFramesAB(&def.base, a, b, anchorA2, anchorB2);
    def.base.localFrameA.q = b2MakeRot(b2Atan2(axis2[1], axis2[0]));
    def.enableSpring = enableSpring != 0;
    def.hertz = hertz;
    def.dampingRatio = dampingRatio;
    def.enableLimit = enableLimit != 0;
    def.lowerTranslation = lower;
    def.upperTranslation = upper;
    def.enableMotor = enableMotor != 0;
    def.motorSpeed = motorSpeed;
    def.maxMotorTorque = maxMotorTorque;
    return b2StoreJointId(b2CreateWheelJoint(jWorld(worldIdRaw), &def));
}

uint64_t b2Glue_CreateMouse(uint32_t worldIdRaw, uint64_t ground, uint64_t b, const float* target2, const float* anchorB2, float maxForce, float hertz, float dampingRatio) {
    b2MotorJointDef def = b2DefaultMotorJointDef();
    def.base.bodyIdA = jBody(ground);
    def.base.bodyIdB = jBody(b);
    // Frame A sits on the target. The linear spring drives frame B's point onto frame A's point.
    def.base.localFrameA.p = b2Body_GetLocalPoint(def.base.bodyIdA, b2ToPos((b2Vec2){ target2[0], target2[1] }));
    def.base.localFrameB.p = (b2Vec2){ anchorB2[0], anchorB2[1] };
    def.base.collideConnected = false;
    def.linearHertz = hertz;
    def.linearDampingRatio = dampingRatio;
    def.maxSpringForce = maxForce;
    return b2StoreJointId(b2CreateMotorJoint(jWorld(worldIdRaw), &def));
}

uint64_t b2Glue_CreateFilter(uint32_t worldIdRaw, uint64_t a, uint64_t b) {
    b2FilterJointDef def = b2DefaultFilterJointDef();
    def.base.bodyIdA = jBody(a);
    def.base.bodyIdB = jBody(b);
    def.base.collideConnected = false;
    return b2StoreJointId(b2CreateFilterJoint(jWorld(worldIdRaw), &def));
}

// ---- Mouse target ----

int b2Glue_MouseSetTarget(uint64_t jointIdRaw, float x, float y) {
    b2JointId joint = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(joint) || b2Joint_GetType(joint) != b2_motorJoint) {
        return 0;
    }
    b2Transform frame = b2Joint_GetLocalFrameA(joint);
    frame.p = b2Body_GetLocalPoint(b2Joint_GetBodyA(joint), b2ToPos((b2Vec2){ x, y }));
    b2Joint_SetLocalFrameA(joint, frame);
    b2Joint_WakeBodies(joint);
    return 1;
}

// ---- Kind ----

// Returns the b2JointType, or -1 for a stale id.
int b2Glue_JointType(uint64_t jointIdRaw) {
    b2JointId j = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(j)) {
        return -1;
    }
    return (int)b2Joint_GetType(j);
}

// ---- Limits, springs, motors ----

int b2Glue_JointSetLimits(uint64_t jointIdRaw, float lower, float upper) {
    b2JointId j = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(j)) return 0;
    switch (b2Joint_GetType(j)) {
    case b2_revoluteJoint: b2RevoluteJoint_SetLimits(j, lower, upper); return 1;
    case b2_prismaticJoint: b2PrismaticJoint_SetLimits(j, lower, upper); return 1;
    case b2_wheelJoint: b2WheelJoint_SetLimits(j, lower, upper); return 1;
    case b2_distanceJoint: b2DistanceJoint_SetLengthRange(j, lower, upper); return 1;
    default: return 0;
    }
}

int b2Glue_JointEnableLimit(uint64_t jointIdRaw, int enable) {
    b2JointId j = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(j)) return 0;
    switch (b2Joint_GetType(j)) {
    case b2_revoluteJoint: b2RevoluteJoint_EnableLimit(j, enable != 0); return 1;
    case b2_prismaticJoint: b2PrismaticJoint_EnableLimit(j, enable != 0); return 1;
    case b2_wheelJoint: b2WheelJoint_EnableLimit(j, enable != 0); return 1;
    case b2_distanceJoint: b2DistanceJoint_EnableLimit(j, enable != 0); return 1;
    default: return 0;
    }
}

int b2Glue_JointEnableSpring(uint64_t jointIdRaw, int enable) {
    b2JointId j = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(j)) return 0;
    switch (b2Joint_GetType(j)) {
    case b2_revoluteJoint: b2RevoluteJoint_EnableSpring(j, enable != 0); return 1;
    case b2_prismaticJoint: b2PrismaticJoint_EnableSpring(j, enable != 0); return 1;
    case b2_wheelJoint: b2WheelJoint_EnableSpring(j, enable != 0); return 1;
    case b2_distanceJoint: b2DistanceJoint_EnableSpring(j, enable != 0); return 1;
    default: return 0;
    }
}

int b2Glue_JointSetSpring(uint64_t jointIdRaw, float hertz, float dampingRatio) {
    b2JointId j = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(j)) return 0;
    switch (b2Joint_GetType(j)) {
    case b2_revoluteJoint:
        b2RevoluteJoint_SetSpringHertz(j, hertz);
        b2RevoluteJoint_SetSpringDampingRatio(j, dampingRatio);
        return 1;
    case b2_prismaticJoint:
        b2PrismaticJoint_SetSpringHertz(j, hertz);
        b2PrismaticJoint_SetSpringDampingRatio(j, dampingRatio);
        return 1;
    case b2_wheelJoint:
        b2WheelJoint_SetSpringHertz(j, hertz);
        b2WheelJoint_SetSpringDampingRatio(j, dampingRatio);
        return 1;
    case b2_distanceJoint:
        b2DistanceJoint_SetSpringHertz(j, hertz);
        b2DistanceJoint_SetSpringDampingRatio(j, dampingRatio);
        return 1;
    default: return 0;
    }
}

int b2Glue_JointEnableMotor(uint64_t jointIdRaw, int enable) {
    b2JointId j = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(j)) return 0;
    switch (b2Joint_GetType(j)) {
    case b2_revoluteJoint: b2RevoluteJoint_EnableMotor(j, enable != 0); return 1;
    case b2_prismaticJoint: b2PrismaticJoint_EnableMotor(j, enable != 0); return 1;
    case b2_wheelJoint: b2WheelJoint_EnableMotor(j, enable != 0); return 1;
    case b2_distanceJoint: b2DistanceJoint_EnableMotor(j, enable != 0); return 1;
    default: return 0;
    }
}

int b2Glue_JointSetMotor(uint64_t jointIdRaw, float speed, float maxForce) {
    b2JointId j = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(j)) return 0;
    switch (b2Joint_GetType(j)) {
    case b2_revoluteJoint:
        b2RevoluteJoint_SetMotorSpeed(j, speed);
        b2RevoluteJoint_SetMaxMotorTorque(j, maxForce);
        return 1;
    case b2_prismaticJoint:
        b2PrismaticJoint_SetMotorSpeed(j, speed);
        b2PrismaticJoint_SetMaxMotorForce(j, maxForce);
        return 1;
    case b2_wheelJoint:
        b2WheelJoint_SetMotorSpeed(j, speed);
        b2WheelJoint_SetMaxMotorTorque(j, maxForce);
        return 1;
    case b2_distanceJoint:
        b2DistanceJoint_SetMotorSpeed(j, speed);
        b2DistanceJoint_SetMaxMotorForce(j, maxForce);
        return 1;
    default: return 0;
    }
}

// ---- Reaction ----

int b2Glue_JointReaction(uint64_t jointIdRaw, float* out3) {
    b2JointId j = jJoint(jointIdRaw);
    if (!b2Joint_IsValid(j)) {
        return 0;
    }
    b2Vec2 force = b2Joint_GetConstraintForce(j);
    out3[0] = force.x;
    out3[1] = force.y;
    out3[2] = b2Joint_GetConstraintTorque(j);
    return 1;
}
