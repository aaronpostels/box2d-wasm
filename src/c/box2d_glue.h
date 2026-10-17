// SPDX-License-Identifier: MIT
// Box2D WebAssembly Glue Header
#pragma once

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"
#include "box2d/id.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define B2_GLUE_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define B2_GLUE_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

// World Management
B2_GLUE_EXPORT uint32_t b2Glue_CreateWorld(float gx, float gy, int enableSleep, int enableContinuous);
B2_GLUE_EXPORT void b2Glue_DestroyWorld(uint32_t worldIdRaw);
B2_GLUE_EXPORT void b2Glue_Step(uint32_t worldIdRaw, float timeStep, int subStepCount);
B2_GLUE_EXPORT bool b2Glue_WorldIsValid(uint32_t worldIdRaw);
B2_GLUE_EXPORT void b2Glue_SetGravity(uint32_t worldIdRaw, float gx, float gy);
B2_GLUE_EXPORT void b2Glue_Configure(uint32_t worldIdRaw, bool sleeping, bool continuous);
B2_GLUE_EXPORT void b2Glue_SetContactTuning(uint32_t worldIdRaw, float hertz, float dampingRatio, float pushSpeed);
B2_GLUE_EXPORT uint64_t b2Glue_StateHash(uint32_t worldIdRaw);
B2_GLUE_EXPORT void b2Glue_SetTaskSystem(uint32_t worldIdRaw, int workers, b2EnqueueTaskCallback* enqueue, b2FinishTaskCallback* finish);
B2_GLUE_EXPORT int b2Glue_WorkerCount(uint32_t worldIdRaw);

// Body Management
B2_GLUE_EXPORT uint64_t b2Glue_CreateBody(uint32_t worldIdRaw, int type, float px, float py, float rotation);
B2_GLUE_EXPORT uint64_t b2Glue_CreateBodyEx(uint32_t worldIdRaw, int type, const float* pose3, const float* vel3, int flags);
B2_GLUE_EXPORT void b2Glue_DestroyBody(uint64_t bodyIdRaw);
B2_GLUE_EXPORT bool b2Glue_BodyIsValid(uint64_t bodyIdRaw);
B2_GLUE_EXPORT void b2Glue_SetTransform(uint64_t bodyIdRaw, float px, float py, float rotation);
B2_GLUE_EXPORT void b2Glue_GetTransform(uint64_t bodyIdRaw, float* out3);
B2_GLUE_EXPORT void b2Glue_SetTargetTransform(uint64_t bodyIdRaw, const float* pose3, float dt);
B2_GLUE_EXPORT void b2Glue_SetLinearVelocity(uint64_t bodyIdRaw, float vx, float vy);
B2_GLUE_EXPORT void b2Glue_GetLinearVelocity(uint64_t bodyIdRaw, float* out2);
B2_GLUE_EXPORT void b2Glue_SetAngularVelocity(uint64_t bodyIdRaw, float omega);
B2_GLUE_EXPORT float b2Glue_GetAngularVelocity(uint64_t bodyIdRaw);
B2_GLUE_EXPORT void b2Glue_ApplyForce(uint64_t bodyIdRaw, float fx, float fy, float px, float py, bool wake);
B2_GLUE_EXPORT void b2Glue_ApplyForceToCenter(uint64_t bodyIdRaw, float fx, float fy, bool wake);
B2_GLUE_EXPORT void b2Glue_ApplyLinearImpulse(uint64_t bodyIdRaw, float ix, float iy, float px, float py, bool wake);
B2_GLUE_EXPORT void b2Glue_ApplyLinearImpulseToCenter(uint64_t bodyIdRaw, float ix, float iy, bool wake);
B2_GLUE_EXPORT void b2Glue_ApplyAngularImpulse(uint64_t bodyIdRaw, float impulse, bool wake);
B2_GLUE_EXPORT float b2Glue_BodyMass(uint64_t bodyIdRaw);
B2_GLUE_EXPORT void b2Glue_SetBodyMass(uint64_t bodyIdRaw, float mass);
B2_GLUE_EXPORT bool b2Glue_BodyIsAwake(uint64_t bodyIdRaw);
B2_GLUE_EXPORT void b2Glue_SetSleepThreshold(uint64_t bodyIdRaw, float threshold);

// Batch and Event State
B2_GLUE_EXPORT void b2Glue_GetBatchTransforms(const uint64_t* ids, int count, float* out3);
B2_GLUE_EXPORT void b2Glue_GetBatchState(const uint64_t* ids, int count, float* out6);
B2_GLUE_EXPORT int b2Glue_MovedBodies(uint32_t worldIdRaw, uint64_t* ids, float* out6, int max);

// Shapes
B2_GLUE_EXPORT uint64_t b2Glue_AddCircle(uint64_t bodyIdRaw, float radius, float density, float friction, float restitution, int flags);
B2_GLUE_EXPORT uint64_t b2Glue_AddBox(uint64_t bodyIdRaw, float hx, float hy, float density, float friction, float restitution, int flags);
B2_GLUE_EXPORT uint64_t b2Glue_AddCapsule(uint64_t bodyIdRaw, float halfHeight, float radius, float density, float friction, float restitution, int flags);
B2_GLUE_EXPORT uint64_t b2Glue_AddSegment(uint64_t bodyIdRaw, float x1, float y1, float x2, float y2, float density, float friction, float restitution, int flags);
B2_GLUE_EXPORT uint64_t b2Glue_AddPolygon(uint64_t bodyIdRaw, const float* points2, int count, float radius, float density, float friction, float restitution, int flags);
B2_GLUE_EXPORT void b2Glue_DestroyShape(uint64_t shapeIdRaw, bool updateBodyMass);
B2_GLUE_EXPORT bool b2Glue_ShapeIsValid(uint64_t shapeIdRaw);
B2_GLUE_EXPORT uint64_t b2Glue_ShapeBody(uint64_t shapeIdRaw);

// Joints
B2_GLUE_EXPORT uint64_t b2Glue_CreateWeld(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, float linearHertz, float angularHertz);
B2_GLUE_EXPORT uint64_t b2Glue_CreateRevolute(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, int limited, float lower, float upper);
B2_GLUE_EXPORT uint64_t b2Glue_CreatePrismatic(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, const float* axis2, int limited, float lower, float upper);
B2_GLUE_EXPORT uint64_t b2Glue_CreateDistance(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, float length, float hertz, float dampingRatio);
B2_GLUE_EXPORT uint64_t b2Glue_CreateMotor(uint32_t worldIdRaw, uint64_t a, uint64_t b, float maxForce, float maxTorque);
B2_GLUE_EXPORT void b2Glue_DestroyJoint(uint64_t jointIdRaw);
B2_GLUE_EXPORT bool b2Glue_JointIsValid(uint64_t jointIdRaw);

// Events
B2_GLUE_EXPORT int b2Glue_ContactBeginEvents(uint32_t worldIdRaw, uint64_t* out2, int max);
B2_GLUE_EXPORT int b2Glue_ContactEndEvents(uint32_t worldIdRaw, uint64_t* out2, int max);
B2_GLUE_EXPORT int b2Glue_SensorBeginEvents(uint32_t worldIdRaw, uint64_t* out2, int max);
B2_GLUE_EXPORT int b2Glue_SensorEndEvents(uint32_t worldIdRaw, uint64_t* out2, int max);

// Queries & Raycast
B2_GLUE_EXPORT uint64_t b2Glue_CastRayClosest(uint32_t worldIdRaw, const float* origin2, const float* translation2, float* outResult6);

// Character Mover
B2_GLUE_EXPORT int b2Glue_MoveCharacter(uint32_t worldIdRaw, uint64_t selfShapeIdRaw, float* pos2, float halfHeight, float radius, const float* delta2, float groundNormalY);

// Snapshots
B2_GLUE_EXPORT int b2Glue_SnapshotSize(uint32_t worldIdRaw);
B2_GLUE_EXPORT int b2Glue_SnapshotTake(uint32_t worldIdRaw, uint8_t* outBuffer, int capacity);
B2_GLUE_EXPORT bool b2Glue_SnapshotRestore(uint32_t worldIdRaw, const uint8_t* buffer, int size);
B2_GLUE_EXPORT uint32_t b2Glue_SnapshotCreateWorld(const uint8_t* buffer, int size, int workerCount);

#ifdef __cplusplus
}
#endif
