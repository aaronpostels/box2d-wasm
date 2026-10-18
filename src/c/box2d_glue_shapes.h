// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: shapes declarations. Exported with B2_GLUE_EXPORT, like box2d_glue.h.
#pragma once

#include "box2d_glue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Shape properties, readable and writable after creation. Ids must be live.
B2_GLUE_EXPORT void b2Glue_ShapeSetFriction(uint64_t shapeIdRaw, float friction);
B2_GLUE_EXPORT float b2Glue_ShapeFriction(uint64_t shapeIdRaw);
B2_GLUE_EXPORT void b2Glue_ShapeSetRestitution(uint64_t shapeIdRaw, float restitution);
B2_GLUE_EXPORT float b2Glue_ShapeRestitution(uint64_t shapeIdRaw);
B2_GLUE_EXPORT void b2Glue_ShapeSetDensity(uint64_t shapeIdRaw, float density, bool updateBodyMass);
B2_GLUE_EXPORT float b2Glue_ShapeDensity(uint64_t shapeIdRaw);
// Box2D has no runtime sensor toggle: a shape's sensor flag is fixed at creation (SHAPE_SENSOR).
B2_GLUE_EXPORT bool b2Glue_ShapeIsSensor(uint64_t shapeIdRaw);
B2_GLUE_EXPORT void b2Glue_ShapeSetRollingResistance(uint64_t shapeIdRaw, float rollingResistance);
B2_GLUE_EXPORT float b2Glue_ShapeRollingResistance(uint64_t shapeIdRaw);
B2_GLUE_EXPORT void b2Glue_ShapeSetTangentSpeed(uint64_t shapeIdRaw, float tangentSpeed);
B2_GLUE_EXPORT float b2Glue_ShapeTangentSpeed(uint64_t shapeIdRaw);
// Corner radius of a polygon shape (0 for other shapes). Rounded polygons come from b2Glue_AddPolygon's radius.
B2_GLUE_EXPORT float b2Glue_ShapeRadius(uint64_t shapeIdRaw);

// Chain shapes: a polyline of one-sided segments (isLoop = 0) or a closed loop (isLoop = 1).
// Each segment collides only on its right-hand side (Box2D's RightPerp of the segment direction), so a clockwise
// loop is solid inside and a right-to-left line is solid on top. Returns 0 when the point count is too small.
B2_GLUE_EXPORT uint64_t b2Glue_AddChain(uint64_t bodyIdRaw, const float* points2, int count, int isLoop, float friction, float restitution, float rollingResistance, float tangentSpeed);
B2_GLUE_EXPORT bool b2Glue_ChainIsValid(uint64_t chainIdRaw);
B2_GLUE_EXPORT int b2Glue_ChainSegmentCount(uint64_t chainIdRaw);

#ifdef __cplusplus
}
#endif
