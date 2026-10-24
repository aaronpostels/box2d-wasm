// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: bodies.
#include "box2d_glue_bodies.h"

static inline b2BodyId bodyOfRaw(uint64_t val) {
    return b2LoadBodyId(val);
}

void b2Glue_SetLinearDamping(uint64_t bodyIdRaw, float damping) {
    b2Body_SetLinearDamping(bodyOfRaw(bodyIdRaw), damping);
}

float b2Glue_GetLinearDamping(uint64_t bodyIdRaw) {
    return b2Body_GetLinearDamping(bodyOfRaw(bodyIdRaw));
}

void b2Glue_SetAngularDamping(uint64_t bodyIdRaw, float damping) {
    b2Body_SetAngularDamping(bodyOfRaw(bodyIdRaw), damping);
}

float b2Glue_GetAngularDamping(uint64_t bodyIdRaw) {
    return b2Body_GetAngularDamping(bodyOfRaw(bodyIdRaw));
}

void b2Glue_SetGravityScale(uint64_t bodyIdRaw, float scale) {
    b2Body_SetGravityScale(bodyOfRaw(bodyIdRaw), scale);
}

float b2Glue_GetGravityScale(uint64_t bodyIdRaw) {
    return b2Body_GetGravityScale(bodyOfRaw(bodyIdRaw));
}

void b2Glue_SetFixedRotation(uint64_t bodyIdRaw, bool fixed) {
    b2BodyId id = bodyOfRaw(bodyIdRaw);
    b2MotionLocks locks = b2Body_GetMotionLocks(id);
    locks.angularZ = fixed;
    b2Body_SetMotionLocks(id, locks);
}

bool b2Glue_GetFixedRotation(uint64_t bodyIdRaw) {
    return b2Body_GetMotionLocks(bodyOfRaw(bodyIdRaw)).angularZ;
}

void b2Glue_SetMotionLocks(uint64_t bodyIdRaw, int bits) {
    b2MotionLocks locks = { .linearX = (bits & 1) != 0, .linearY = (bits & 2) != 0, .angularZ = (bits & 4) != 0 };
    b2Body_SetMotionLocks(bodyOfRaw(bodyIdRaw), locks);
}

int b2Glue_GetMotionLocks(uint64_t bodyIdRaw) {
    b2MotionLocks l = b2Body_GetMotionLocks(bodyOfRaw(bodyIdRaw));
    return (l.linearX ? 1 : 0) | (l.linearY ? 2 : 0) | (l.angularZ ? 4 : 0);
}

void b2Glue_SetBullet(uint64_t bodyIdRaw, bool bullet) {
    b2Body_SetBullet(bodyOfRaw(bodyIdRaw), bullet);
}

bool b2Glue_IsBullet(uint64_t bodyIdRaw) {
    return b2Body_IsBullet(bodyOfRaw(bodyIdRaw));
}

void b2Glue_GetMassData(uint64_t bodyIdRaw, float* out4) {
    b2MassData m = b2Body_GetMassData(bodyOfRaw(bodyIdRaw));
    out4[0] = m.mass;
    out4[1] = m.center.x;
    out4[2] = m.center.y;
    out4[3] = m.rotationalInertia;
}

void b2Glue_SetMassData(uint64_t bodyIdRaw, const float* in4) {
    b2MassData m;
    m.mass = in4[0];
    m.center = (b2Vec2){ in4[1], in4[2] };
    m.rotationalInertia = in4[3];
    b2Body_SetMassData(bodyOfRaw(bodyIdRaw), m);
}

void b2Glue_SetBodyUserData(uint64_t bodyIdRaw, uint32_t value) {
    b2Body_SetUserData(bodyOfRaw(bodyIdRaw), (void*)(uintptr_t)value);
}

uint32_t b2Glue_GetBodyUserData(uint64_t bodyIdRaw) {
    return (uint32_t)(uintptr_t)b2Body_GetUserData(bodyOfRaw(bodyIdRaw));
}

void b2Glue_EnableBody(uint64_t bodyIdRaw) {
    b2Body_Enable(bodyOfRaw(bodyIdRaw));
}

void b2Glue_DisableBody(uint64_t bodyIdRaw) {
    b2Body_Disable(bodyOfRaw(bodyIdRaw));
}

bool b2Glue_BodyIsEnabled(uint64_t bodyIdRaw) {
    return b2Body_IsEnabled(bodyOfRaw(bodyIdRaw));
}

void b2Glue_ApplyLinearImpulseBatch(const uint64_t* ids, int count, const float* impulses2, bool wake) {
    for (int i = 0; i < count; ++i) {
        b2Body_ApplyLinearImpulseToCenter(bodyOfRaw(ids[i]), (b2Vec2){ impulses2[2 * i], impulses2[2 * i + 1] }, wake);
    }
}

void b2Glue_ApplyAngularImpulseBatch(const uint64_t* ids, int count, const float* angular1, bool wake) {
    for (int i = 0; i < count; ++i) {
        b2Body_ApplyAngularImpulse(bodyOfRaw(ids[i]), angular1[i], wake);
    }
}
