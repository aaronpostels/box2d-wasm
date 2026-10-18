// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: shape filter declarations. Exported with B2_GLUE_EXPORT, like box2d_glue.h.
//
// A shape's filter decides which shapes it collides with: two shapes collide when each one's category
// bits match the other's mask bits and they do not share a nonzero group index (a positive group forces
// collision, a negative group forces no collision).
#pragma once

#include "box2d_glue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Set the category bits, mask bits and group index of a shape. Box2D rebuilds its contacts for the shape.
B2_GLUE_EXPORT void b2Glue_SetShapeFilter(uint64_t shapeIdRaw, uint64_t categoryBits, uint64_t maskBits, int groupIndex);

// Read the filter back. out2 receives categoryBits and maskBits, outGroup receives groupIndex.
B2_GLUE_EXPORT void b2Glue_GetShapeFilter(uint64_t shapeIdRaw, uint64_t* out2, int* outGroup);

#ifdef __cplusplus
}
#endif
