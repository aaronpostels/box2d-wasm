// SPDX-License-Identifier: MIT
// Box2D WebAssembly glue: queries. Shape cast, overlaps (AABB, circle, polygon), all-hits ray cast, explosion.
// Overlaps and shape casts use Box2D's own proxy tests, so a query shape is always a point cloud plus a radius.
#include "box2d_glue_queries.h"

// Shape proxy from 1..B2_MAX_POLYGON_VERTICES points. Returns false when the count is out of range.
static bool makeProxy(const float* points2, int count, float radius, b2ShapeProxy* proxy) {
    if (count < 1 || count > B2_MAX_POLYGON_VERTICES) return false;
    b2Vec2 points[B2_MAX_POLYGON_VERTICES];
    for (int i = 0; i < count; ++i) {
        points[i] = (b2Vec2){ points2[2 * i], points2[2 * i + 1] };
    }
    *proxy = b2MakeProxy(points, count, radius);
    return true;
}

// ---- Shape cast ----

typedef struct {
    b2Glue_Hit hit;
    bool found;
} ClosestCast;

// Returning the fraction clips the cast, so the last call is the closest hit.
static float closestCastCallback(b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context) {
    ClosestCast* c = context;
    if (!c->found || fraction < c->hit.fraction) {
        b2Vec2 p = b2ToVec2(point);
        c->found = true;
        c->hit.shape = b2StoreShapeId(shapeId);
        c->hit.fraction = fraction;
        c->hit.point[0] = p.x;
        c->hit.point[1] = p.y;
        c->hit.normal[0] = normal.x;
        c->hit.normal[1] = normal.y;
    }
    return fraction;
}

int b2Glue_CastShapeClosest(uint32_t worldIdRaw, const float* points2, int count, float radius, const float* origin2, const float* translation2, b2Glue_Hit* out) {
    b2ShapeProxy proxy;
    if (!makeProxy(points2, count, radius, &proxy)) return -1;
    ClosestCast c = { 0 };
    b2World_CastShape(b2LoadWorldId(worldIdRaw), b2ToPos((b2Vec2){ origin2[0], origin2[1] }), &proxy,
                      (b2Vec2){ translation2[0], translation2[1] }, b2DefaultQueryFilter(), closestCastCallback, &c);
    if (!c.found) return 0;
    if (out) *out = c.hit;
    return 1;
}

// ---- Overlap ----

typedef struct {
    uint64_t* out;
    int max;
    int count;
} OverlapList;

static bool overlapCallback(b2ShapeId shapeId, void* context) {
    OverlapList* c = context;
    if (c->out && c->count < c->max) c->out[c->count] = b2StoreShapeId(shapeId);
    c->count++;
    return true;
}

static int overlapProxy(uint32_t worldIdRaw, const b2ShapeProxy* proxy, uint64_t* out, int max) {
    OverlapList list = { out, max, 0 };
    b2World_OverlapShape(b2LoadWorldId(worldIdRaw), b2ToPos((b2Vec2){ 0.0f, 0.0f }), proxy, b2DefaultQueryFilter(), overlapCallback, &list);
    return list.count;
}

int b2Glue_OverlapAABB(uint32_t worldIdRaw, float minX, float minY, float maxX, float maxY, uint64_t* out, int max) {
    float corners[8] = { minX, minY, maxX, minY, maxX, maxY, minX, maxY };
    b2ShapeProxy proxy;
    makeProxy(corners, 4, 0.0f, &proxy);
    return overlapProxy(worldIdRaw, &proxy, out, max);
}

int b2Glue_OverlapCircle(uint32_t worldIdRaw, float cx, float cy, float radius, uint64_t* out, int max) {
    float center[2] = { cx, cy };
    b2ShapeProxy proxy;
    makeProxy(center, 1, radius, &proxy);
    return overlapProxy(worldIdRaw, &proxy, out, max);
}

int b2Glue_OverlapPolygon(uint32_t worldIdRaw, const float* points2, int count, float radius, uint64_t* out, int max) {
    b2ShapeProxy proxy;
    if (!makeProxy(points2, count, radius, &proxy)) return -1;
    return overlapProxy(worldIdRaw, &proxy, out, max);
}

// ---- Ray cast, all hits ----

typedef struct {
    b2Glue_Hit* out;
    int max;
    int count;
} AllHits;

// Returning 1 keeps the ray unclipped, so every shape on the path is reported. Box2D calls back in no fixed order, so
// once the buffer is full a hit replaces the farthest stored hit when it is nearer. The kept hits are the nearest max.
static float allHitsCallback(b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context) {
    AllHits* c = context;
    if (c->out && c->max > 0) {
        int stored = c->count < c->max ? c->count : c->max;
        int slot = stored;
        if (stored == c->max) {
            int farthest = 0;
            for (int i = 1; i < stored; ++i) {
                if (c->out[i].fraction > c->out[farthest].fraction) farthest = i;
            }
            if (fraction >= c->out[farthest].fraction) {
                c->count++;
                return 1.0f;
            }
            slot = farthest;
        }
        b2Vec2 p = b2ToVec2(point);
        b2Glue_Hit* h = c->out + slot;
        h->shape = b2StoreShapeId(shapeId);
        h->fraction = fraction;
        h->point[0] = p.x;
        h->point[1] = p.y;
        h->normal[0] = normal.x;
        h->normal[1] = normal.y;
    }
    c->count++;
    return 1.0f;
}

int b2Glue_CastRayAll(uint32_t worldIdRaw, const float* origin2, const float* translation2, b2Glue_Hit* out, int max) {
    AllHits all = { out, max, 0 };
    b2World_CastRay(b2LoadWorldId(worldIdRaw), b2ToPos((b2Vec2){ origin2[0], origin2[1] }),
                    (b2Vec2){ translation2[0], translation2[1] }, b2DefaultQueryFilter(), allHitsCallback, &all);
    // Insertion sort by fraction, nearest first, as in box3d_glue_queries.c. Only the stored entries are sorted.
    int n = all.count < max ? all.count : max;
    for (int i = 1; out && i < n; ++i) {
        b2Glue_Hit h = out[i];
        int j = i - 1;
        while (j >= 0 && out[j].fraction > h.fraction) {
            out[j + 1] = out[j];
            --j;
        }
        out[j + 1] = h;
    }
    return all.count;
}

// ---- Explosion ----

void b2Glue_Explode(uint32_t worldIdRaw, float px, float py, float radius, float falloff, float impulsePerLength, uint64_t maskBits) {
    b2ExplosionDef def = b2DefaultExplosionDef();
    def.maskBits = maskBits;
    def.position = b2ToPos((b2Vec2){ px, py });
    def.radius = radius;
    def.falloff = falloff;
    def.impulsePerLength = impulsePerLength;
    b2World_Explode(b2LoadWorldId(worldIdRaw), &def);
}
