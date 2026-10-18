// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: shapes. Property get/set on live shapes, rounded polygon radius, chain shapes.
#include "box2d_glue_shapes.h"

static inline b2ShapeId shapeOf(uint64_t val) {
    return b2LoadShapeId(val);
}

static inline b2BodyId bodyOf(uint64_t val) {
    return b2LoadBodyId(val);
}

static inline b2ChainId chainOf(uint64_t val) {
    return b2LoadChainId(val);
}

// Friction, restitution, rolling resistance and tangent speed live in one surface material. Rolling resistance and
// tangent speed have no setter of their own, so they go through get and set of the material.

void b2Glue_ShapeSetFriction(uint64_t shapeIdRaw, float friction) {
    b2Shape_SetFriction(shapeOf(shapeIdRaw), friction);
}

float b2Glue_ShapeFriction(uint64_t shapeIdRaw) {
    return b2Shape_GetFriction(shapeOf(shapeIdRaw));
}

void b2Glue_ShapeSetRestitution(uint64_t shapeIdRaw, float restitution) {
    b2Shape_SetRestitution(shapeOf(shapeIdRaw), restitution);
}

float b2Glue_ShapeRestitution(uint64_t shapeIdRaw) {
    return b2Shape_GetRestitution(shapeOf(shapeIdRaw));
}

void b2Glue_ShapeSetDensity(uint64_t shapeIdRaw, float density, bool updateBodyMass) {
    b2Shape_SetDensity(shapeOf(shapeIdRaw), density, updateBodyMass);
}

float b2Glue_ShapeDensity(uint64_t shapeIdRaw) {
    return b2Shape_GetDensity(shapeOf(shapeIdRaw));
}

bool b2Glue_ShapeIsSensor(uint64_t shapeIdRaw) {
    return b2Shape_IsSensor(shapeOf(shapeIdRaw));
}

void b2Glue_ShapeSetRollingResistance(uint64_t shapeIdRaw, float rollingResistance) {
    b2ShapeId id = shapeOf(shapeIdRaw);
    b2SurfaceMaterial material = b2Shape_GetSurfaceMaterial(id);
    material.rollingResistance = rollingResistance;
    b2Shape_SetSurfaceMaterial(id, &material);
}

float b2Glue_ShapeRollingResistance(uint64_t shapeIdRaw) {
    return b2Shape_GetSurfaceMaterial(shapeOf(shapeIdRaw)).rollingResistance;
}

void b2Glue_ShapeSetTangentSpeed(uint64_t shapeIdRaw, float tangentSpeed) {
    b2ShapeId id = shapeOf(shapeIdRaw);
    b2SurfaceMaterial material = b2Shape_GetSurfaceMaterial(id);
    material.tangentSpeed = tangentSpeed;
    b2Shape_SetSurfaceMaterial(id, &material);
}

float b2Glue_ShapeTangentSpeed(uint64_t shapeIdRaw) {
    return b2Shape_GetSurfaceMaterial(shapeOf(shapeIdRaw)).tangentSpeed;
}

float b2Glue_ShapeRadius(uint64_t shapeIdRaw) {
    b2ShapeId id = shapeOf(shapeIdRaw);
    if (b2Shape_GetType(id) != b2_polygonShape) return 0.0f;
    return b2Shape_GetPolygon(id).radius;
}

uint64_t b2Glue_AddChain(uint64_t bodyIdRaw, const float* points2, int count, int isLoop, float friction, float restitution, float rollingResistance, float tangentSpeed) {
    if (points2 == NULL || count < (isLoop ? 3 : 2)) return 0;
    // One material for the whole chain. Box2D clones the points and the material, so locals are fine.
    b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
    material.friction = friction;
    material.restitution = restitution;
    material.rollingResistance = rollingResistance;
    material.tangentSpeed = tangentSpeed;
    b2ChainDef def = b2DefaultChainDef();
    def.points = (const b2Vec2*)points2;
    def.pointCount = count;
    def.materials = &material;
    def.materialCount = 1;
    def.isLoop = isLoop != 0;
    if (!def.isLoop) {
        // An open chain needs ghost points past each end so its end segments are smooth. Extrapolate them.
        const b2Vec2* p = def.points;
        def.ghost1 = (b2Vec2){ 2.0f * p[0].x - p[1].x, 2.0f * p[0].y - p[1].y };
        def.ghost2 = (b2Vec2){ 2.0f * p[count - 1].x - p[count - 2].x, 2.0f * p[count - 1].y - p[count - 2].y };
    }
    return b2StoreChainId(b2CreateChain(bodyOf(bodyIdRaw), &def));
}

bool b2Glue_ChainIsValid(uint64_t chainIdRaw) {
    return b2Chain_IsValid(chainOf(chainIdRaw));
}

int b2Glue_ChainSegmentCount(uint64_t chainIdRaw) {
    return b2Chain_GetSegmentCount(chainOf(chainIdRaw));
}
