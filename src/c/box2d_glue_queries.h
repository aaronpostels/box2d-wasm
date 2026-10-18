// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: queries declarations. Exported with B2_GLUE_EXPORT, like box2d_glue.h.
#pragma once

#include "box2d_glue.h"

#ifdef __cplusplus
extern "C" {
#endif

// One query hit. Layout is fixed (32 bytes, 8-byte aligned, matches QueryHit in crates/box2d-sys/src/queries.rs).
typedef struct b2Glue_Hit {
    uint64_t shape;      // shape id
    float fraction;      // ray or cast fraction along the translation, 0..1
    float point[2];      // world hit point
    float normal[2];     // surface normal at the hit
} b2Glue_Hit;

// Closest hit of a shape cast. The proxy is a point cloud (1..8 points) with a radius, relative to origin2.
// Returns 1 and fills *out on a hit, 0 on a miss, -1 when count is out of range.
B2_GLUE_EXPORT int b2Glue_CastShapeClosest(uint32_t worldIdRaw, const float* points2, int count, float radius, const float* origin2, const float* translation2, b2Glue_Hit* out);

// Overlap queries. Each returns the total number of shapes that overlap. Only the first min(total, max) ids are
// written to out, in no particular order. max may be 0 with out NULL to just count.
B2_GLUE_EXPORT int b2Glue_OverlapAABB(uint32_t worldIdRaw, float minX, float minY, float maxX, float maxY, uint64_t* out, int max);
B2_GLUE_EXPORT int b2Glue_OverlapCircle(uint32_t worldIdRaw, float cx, float cy, float radius, uint64_t* out, int max);
// Returns -1 when count is out of range (1..8 points), otherwise the total as above.
B2_GLUE_EXPORT int b2Glue_OverlapPolygon(uint32_t worldIdRaw, const float* points2, int count, float radius, uint64_t* out, int max);

// All hits of a ray, sorted by fraction, nearest first. Returns the total hit count. Only the first min(total, max)
// hits are written to out. When total exceeds max, the kept hits are the nearest max. max may be 0 with out NULL to
// just count.
B2_GLUE_EXPORT int b2Glue_CastRayAll(uint32_t worldIdRaw, const float* origin2, const float* translation2, b2Glue_Hit* out, int max);

// Radial impulse: every awake dynamic shape within radius + falloff gets an impulse, full strength inside radius and
// linear falloff to zero at radius + falloff. impulsePerLength scales the impulse per unit of projected perimeter.
B2_GLUE_EXPORT void b2Glue_Explode(uint32_t worldIdRaw, float px, float py, float radius, float falloff, float impulsePerLength, uint64_t maskBits);

#ifdef __cplusplus
}
#endif
