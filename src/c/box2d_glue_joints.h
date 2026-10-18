// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: joints. Wheel, mouse and filter creation, limits, springs and motors for the kinds that have
// them, and joint reaction forces. Weld, revolute, prismatic, distance and motor creation stay in box2d_glue.c.
// Exported with B2_GLUE_EXPORT, like box2d_glue.h.
#pragma once

#include "box2d_glue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Creation. Box2D asserts on bad ids, so a returned id is always a live joint.
// Wheel: body B moves along the x axis of frame A (axis2 sets it). Spring, limit and motor are optional (enable flags).
B2_GLUE_EXPORT uint64_t b2Glue_CreateWheel(uint32_t worldIdRaw, uint64_t a, uint64_t b, const float* anchorA2, const float* anchorB2, const float* axis2, int enableSpring, float hertz, float dampingRatio, int enableLimit, float lower, float upper, int enableMotor, float motorSpeed, float maxMotorTorque);
// Mouse: Box2D 3.2 has no mouse joint, so this is a motor joint with a linear spring and no velocity motor. Anchor B2
// (local to B) is pulled to target2 (world). Pass the ground body as a.
B2_GLUE_EXPORT uint64_t b2Glue_CreateMouse(uint32_t worldIdRaw, uint64_t ground, uint64_t b, const float* target2, const float* anchorB2, float maxForce, float hertz, float dampingRatio);
// Filter: disables collision between a and b and keeps them in one island. No other effect.
B2_GLUE_EXPORT uint64_t b2Glue_CreateFilter(uint32_t worldIdRaw, uint64_t a, uint64_t b);

// Mouse target: moves the pulled point to target2 (world) and wakes the bodies. Returns 0 when the joint is not a motor joint.
B2_GLUE_EXPORT int b2Glue_MouseSetTarget(uint64_t jointIdRaw, float x, float y);

// Kind of joint: the b2JointType value (distance 0, filter 1, motor 2, mover 3, pogo 4, prismatic 5, revolute 6, weld 7, wheel 8).
B2_GLUE_EXPORT int b2Glue_JointType(uint64_t jointIdRaw);

// Limits: revolute (angle), prismatic and wheel (translation), distance (length range). Return 1 when the kind has
// limits, 0 otherwise.
B2_GLUE_EXPORT int b2Glue_JointSetLimits(uint64_t jointIdRaw, float lower, float upper);
B2_GLUE_EXPORT int b2Glue_JointEnableLimit(uint64_t jointIdRaw, int enable);

// Springs: distance, prismatic, revolute and wheel. Return 1 when the kind has a spring, 0 otherwise.
B2_GLUE_EXPORT int b2Glue_JointEnableSpring(uint64_t jointIdRaw, int enable);
B2_GLUE_EXPORT int b2Glue_JointSetSpring(uint64_t jointIdRaw, float hertz, float dampingRatio);

// Motors: distance, prismatic, revolute and wheel. Speed is rad/s or m/s. maxForce is a force for prismatic and distance,
// a torque for revolute and wheel. Return 1 when the kind has a motor, 0 otherwise.
B2_GLUE_EXPORT int b2Glue_JointEnableMotor(uint64_t jointIdRaw, int enable);
B2_GLUE_EXPORT int b2Glue_JointSetMotor(uint64_t jointIdRaw, float speed, float maxForce);

// Reaction on body B from the last step: out3 gets [force x, force y, torque]. Returns 0 for an invalid joint.
B2_GLUE_EXPORT int b2Glue_JointReaction(uint64_t jointIdRaw, float* out3);

#ifdef __cplusplus
}
#endif
