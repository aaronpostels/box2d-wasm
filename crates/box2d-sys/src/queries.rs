//! queries: Rust side of the queries glue (src/c/box2d_glue_queries.c). Shape cast, overlaps, all-hits ray cast and
//! explosion. The extern block is raw; the functions below it take slices and check sizes.
use crate::{Shape, World};

/// One query hit, same layout as `b2Glue_Hit` (32 bytes).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct QueryHit {
    pub shape: Shape,
    pub fraction: f32,
    pub point: [f32; 2],
    pub normal: [f32; 2],
}

/// Box2D polygon proxies hold at most this many points.
pub const MAX_PROXY_POINTS: usize = 8;

extern "C" {
    pub fn b2Glue_CastShapeClosest(world: World, points2: *const f32, count: i32, radius: f32, origin2: *const f32, translation2: *const f32, out: *mut QueryHit) -> i32;
    pub fn b2Glue_OverlapAABB(world: World, min_x: f32, min_y: f32, max_x: f32, max_y: f32, out: *mut Shape, max: i32) -> i32;
    pub fn b2Glue_OverlapCircle(world: World, cx: f32, cy: f32, radius: f32, out: *mut Shape, max: i32) -> i32;
    pub fn b2Glue_OverlapPolygon(world: World, points2: *const f32, count: i32, radius: f32, out: *mut Shape, max: i32) -> i32;
    pub fn b2Glue_CastRayAll(world: World, origin2: *const f32, translation2: *const f32, out: *mut QueryHit, max: i32) -> i32;
    pub fn b2Glue_Explode(world: World, px: f32, py: f32, radius: f32, falloff: f32, impulse_per_length: f32, mask_bits: u64);
}

/// Closest hit of a shape cast. The proxy is 1..=8 points with a radius, placed at `origin` and moved by `translation`.
/// Returns `None` on a miss or when the point count is out of range.
pub fn cast_shape_closest(world: World, points: &[[f32; 2]], radius: f32, origin: [f32; 2], translation: [f32; 2]) -> Option<QueryHit> {
    if points.is_empty() || points.len() > MAX_PROXY_POINTS {
        return None;
    }
    let mut hit = QueryHit::default();
    let found = unsafe {
        b2Glue_CastShapeClosest(world, points.as_ptr() as *const f32, points.len() as i32, radius, origin.as_ptr(), translation.as_ptr(), &mut hit)
    };
    (found == 1).then_some(hit)
}

/// Shapes whose geometry overlaps the box. Writes up to `out.len()` ids and returns the total count.
pub fn overlap_aabb(world: World, min: [f32; 2], max: [f32; 2], out: &mut [Shape]) -> usize {
    unsafe { b2Glue_OverlapAABB(world, min[0], min[1], max[0], max[1], out.as_mut_ptr(), out.len() as i32) as usize }
}

/// Shapes whose geometry overlaps the circle. Writes up to `out.len()` ids and returns the total count.
pub fn overlap_circle(world: World, center: [f32; 2], radius: f32, out: &mut [Shape]) -> usize {
    unsafe { b2Glue_OverlapCircle(world, center[0], center[1], radius, out.as_mut_ptr(), out.len() as i32) as usize }
}

/// Shapes whose geometry overlaps the convex hull of `points` grown by `radius`. Returns `None` when the point count
/// is out of range. Writes up to `out.len()` ids and returns the total count.
pub fn overlap_polygon(world: World, points: &[[f32; 2]], radius: f32, out: &mut [Shape]) -> Option<usize> {
    if points.is_empty() || points.len() > MAX_PROXY_POINTS {
        return None;
    }
    let n = unsafe { b2Glue_OverlapPolygon(world, points.as_ptr() as *const f32, points.len() as i32, radius, out.as_mut_ptr(), out.len() as i32) };
    Some(n as usize)
}

/// Every shape the ray `origin + t * translation` passes through, sorted by fraction, nearest first. Writes up to
/// `out.len()` hits and returns the total count, so a return value above `out.len()` means some hits were dropped.
/// When hits are dropped, the kept ones are the nearest `out.len()`.
pub fn cast_ray_all(world: World, origin: [f32; 2], translation: [f32; 2], out: &mut [QueryHit]) -> usize {
    unsafe { b2Glue_CastRayAll(world, origin.as_ptr(), translation.as_ptr(), out.as_mut_ptr(), out.len() as i32) as usize }
}

/// Radial impulse at `center`. Shapes inside `radius` get the full impulse, shapes between `radius` and
/// `radius + falloff` get a linearly reduced one, shapes further out get none. `mask_bits` filters shapes.
pub fn explode(world: World, center: [f32; 2], radius: f32, falloff: f32, impulse_per_length: f32, mask_bits: u64) {
    unsafe { b2Glue_Explode(world, center[0], center[1], radius, falloff, impulse_per_length, mask_bits) }
}
