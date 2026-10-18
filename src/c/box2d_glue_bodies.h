// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: bodies declarations. Exported with B2_GLUE_EXPORT, like box2d_glue.h.
//
// User data is a 32-bit value: it must round-trip through a wasm32 pointer, so Rust and JS callers
// pass an index or handle (for example an entity id), not a pointer.
#pragma once

#include "box2d_glue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Damping and gravity
B2_GLUE_EXPORT void b2Glue_SetLinearDamping(uint64_t bodyIdRaw, float damping);
B2_GLUE_EXPORT float b2Glue_GetLinearDamping(uint64_t bodyIdRaw);
B2_GLUE_EXPORT void b2Glue_SetAngularDamping(uint64_t bodyIdRaw, float damping);
B2_GLUE_EXPORT float b2Glue_GetAngularDamping(uint64_t bodyIdRaw);
B2_GLUE_EXPORT void b2Glue_SetGravityScale(uint64_t bodyIdRaw, float scale);
B2_GLUE_EXPORT float b2Glue_GetGravityScale(uint64_t bodyIdRaw);

// Fixed rotation: locks angular z and keeps the linear locks as they are.
B2_GLUE_EXPORT void b2Glue_SetFixedRotation(uint64_t bodyIdRaw, bool fixed);
B2_GLUE_EXPORT bool b2Glue_GetFixedRotation(uint64_t bodyIdRaw);

// Mass data: out4 / in4 are mass, centerX, centerY, rotationalInertia.
B2_GLUE_EXPORT void b2Glue_GetMassData(uint64_t bodyIdRaw, float* out4);
B2_GLUE_EXPORT void b2Glue_SetMassData(uint64_t bodyIdRaw, const float* in4);

// User data (32-bit, see the note above).
B2_GLUE_EXPORT void b2Glue_SetBodyUserData(uint64_t bodyIdRaw, uint32_t value);
B2_GLUE_EXPORT uint32_t b2Glue_GetBodyUserData(uint64_t bodyIdRaw);

// Enable and disable: a disabled body takes no part in the simulation and has no contacts.
B2_GLUE_EXPORT void b2Glue_EnableBody(uint64_t bodyIdRaw);
B2_GLUE_EXPORT void b2Glue_DisableBody(uint64_t bodyIdRaw);
B2_GLUE_EXPORT bool b2Glue_BodyIsEnabled(uint64_t bodyIdRaw);

// Impulses by batch. ids has count entries. impulses2 is 2 floats per body (x, y), applied at the center of mass.
// angular1 is 1 float per body, applied as an angular impulse.
B2_GLUE_EXPORT void b2Glue_ApplyLinearImpulseBatch(const uint64_t* ids, int count, const float* impulses2, bool wake);
B2_GLUE_EXPORT void b2Glue_ApplyAngularImpulseBatch(const uint64_t* ids, int count, const float* angular1, bool wake);

#ifdef __cplusplus
}
#endif
