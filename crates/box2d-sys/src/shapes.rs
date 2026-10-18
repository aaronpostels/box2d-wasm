//! shapes: Rust side of the shapes glue (src/c/box2d_glue_shapes.c). Externs are declared here; the safe wrappers check
//! the id in debug builds only, the same contract as the rest of the flat API (a stale id is undefined behaviour in release).
use crate::{Body, Shape};

pub type Chain = u64;

extern "C" {
    pub fn b2Glue_ShapeSetFriction(shape: Shape, friction: f32);
    pub fn b2Glue_ShapeFriction(shape: Shape) -> f32;
    pub fn b2Glue_ShapeSetRestitution(shape: Shape, restitution: f32);
    pub fn b2Glue_ShapeRestitution(shape: Shape) -> f32;
    pub fn b2Glue_ShapeSetDensity(shape: Shape, density: f32, update_body_mass: bool);
    pub fn b2Glue_ShapeDensity(shape: Shape) -> f32;
    pub fn b2Glue_ShapeIsSensor(shape: Shape) -> bool;
    pub fn b2Glue_ShapeSetRollingResistance(shape: Shape, rolling_resistance: f32);
    pub fn b2Glue_ShapeRollingResistance(shape: Shape) -> f32;
    pub fn b2Glue_ShapeSetTangentSpeed(shape: Shape, tangent_speed: f32);
    pub fn b2Glue_ShapeTangentSpeed(shape: Shape) -> f32;
    pub fn b2Glue_ShapeRadius(shape: Shape) -> f32;

    /// `points2` is `count` pairs of x, y. A loop needs at least 3 points, an open chain at least 2.
    pub fn b2Glue_AddChain(body: Body, points2: *const f32, count: i32, is_loop: i32, friction: f32, restitution: f32, rolling_resistance: f32, tangent_speed: f32) -> Chain;
    pub fn b2Glue_ChainIsValid(chain: Chain) -> bool;
    pub fn b2Glue_ChainSegmentCount(chain: Chain) -> i32;
}

fn shape_is_valid(shape: Shape) -> bool {
    unsafe { crate::b2Glue_ShapeIsValid(shape) }
}

pub fn friction(shape: Shape) -> f32 {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeFriction(shape) }
}

pub fn set_friction(shape: Shape, friction: f32) {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeSetFriction(shape, friction) }
}

pub fn restitution(shape: Shape) -> f32 {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeRestitution(shape) }
}

pub fn set_restitution(shape: Shape, restitution: f32) {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeSetRestitution(shape, restitution) }
}

pub fn density(shape: Shape) -> f32 {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeDensity(shape) }
}

/// `update_body_mass` recomputes the body's mass and inertia from its shapes.
pub fn set_density(shape: Shape, density: f32, update_body_mass: bool) {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeSetDensity(shape, density, update_body_mass) }
}

pub fn is_sensor(shape: Shape) -> bool {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeIsSensor(shape) }
}

pub fn rolling_resistance(shape: Shape) -> f32 {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeRollingResistance(shape) }
}

pub fn set_rolling_resistance(shape: Shape, rolling_resistance: f32) {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeSetRollingResistance(shape, rolling_resistance) }
}

/// Surface speed along the contact, as on a conveyor belt. Positive or negative moves bodies along the surface.
pub fn tangent_speed(shape: Shape) -> f32 {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeTangentSpeed(shape) }
}

pub fn set_tangent_speed(shape: Shape, tangent_speed: f32) {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeSetTangentSpeed(shape, tangent_speed) }
}

/// Corner radius of a polygon (0 for other shapes).
pub fn radius(shape: Shape) -> f32 {
    debug_assert!(shape_is_valid(shape));
    unsafe { b2Glue_ShapeRadius(shape) }
}

/// A closed loop when `is_loop`, otherwise an open chain. Segments collide on their right-hand side only: a clockwise
/// loop is solid inside, and reversing the points flips the solid side.
/// Returns `None` when the glue rejects the points (too few, or the body id is invalid).
pub fn add_chain(body: Body, points: &[[f32; 2]], is_loop: bool, friction: f32, restitution: f32, rolling_resistance: f32, tangent_speed: f32) -> Option<Chain> {
    let chain = unsafe {
        b2Glue_AddChain(body, points.as_ptr() as *const f32, points.len() as i32, is_loop as i32, friction, restitution, rolling_resistance, tangent_speed)
    };
    (chain != 0).then_some(chain)
}

pub fn chain_is_valid(chain: Chain) -> bool {
    unsafe { b2Glue_ChainIsValid(chain) }
}

/// Segments in a loop equal the point count; an open chain has one fewer.
pub fn chain_segment_count(chain: Chain) -> i32 {
    debug_assert!(chain_is_valid(chain));
    unsafe { b2Glue_ChainSegmentCount(chain) }
}
