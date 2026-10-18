// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: shape filters.
#include "box2d_glue_filters.h"

static inline b2ShapeId shapeOfRaw(uint64_t val) {
    return b2LoadShapeId(val);
}

void b2Glue_SetShapeFilter(uint64_t shapeIdRaw, uint64_t categoryBits, uint64_t maskBits, int groupIndex) {
    b2Filter f;
    f.categoryBits = categoryBits;
    f.maskBits = maskBits;
    f.groupIndex = groupIndex;
    b2Shape_SetFilter(shapeOfRaw(shapeIdRaw), f);
}

void b2Glue_GetShapeFilter(uint64_t shapeIdRaw, uint64_t* out2, int* outGroup) {
    b2Filter f = b2Shape_GetFilter(shapeOfRaw(shapeIdRaw));
    out2[0] = f.categoryBits;
    out2[1] = f.maskBits;
    *outGroup = f.groupIndex;
}
