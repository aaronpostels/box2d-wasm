//! joints: Rust side of the joints glue (src/c/box2d_glue_joints.c). Extern declarations for the new joint functions and
//! safe wrappers. Weld, revolute, prismatic, distance and motor creation are in lib.rs.
//! The safe wrappers take raw ids, like the rest of this crate: a stale id is a Box2D assert, not undefined behaviour
//! in Rust.
use crate::{Body, Joint, World};

/// Joint kinds, matching `b2JointType`.
pub const JOINT_DISTANCE: i32 = 0;
pub const JOINT_FILTER: i32 = 1;
pub const JOINT_MOTOR: i32 = 2;
pub const JOINT_MOVER: i32 = 3;
pub const JOINT_POGO: i32 = 4;
pub const JOINT_PRISMATIC: i32 = 5;
pub const JOINT_REVOLUTE: i32 = 6;
pub const JOINT_WELD: i32 = 7;
pub const JOINT_WHEEL: i32 = 8;

extern "C" {
    pub fn b2Glue_CreateWheel(world: World, a: Body, b: Body, anchor_a2: *const f32, anchor_b2: *const f32, axis2: *const f32, enable_spring: i32, hertz: f32, damping_ratio: f32, enable_limit: i32, lower: f32, upper: f32, enable_motor: i32, motor_speed: f32, max_motor_torque: f32) -> Joint;
    pub fn b2Glue_CreateMouse(world: World, ground: Body, b: Body, target2: *const f32, anchor_b2: *const f32, max_force: f32, hertz: f32, damping_ratio: f32) -> Joint;
    pub fn b2Glue_CreateFilter(world: World, a: Body, b: Body) -> Joint;
    pub fn b2Glue_MouseSetTarget(joint: Joint, x: f32, y: f32) -> i32;
    pub fn b2Glue_JointType(joint: Joint) -> i32;
    pub fn b2Glue_JointSetLimits(joint: Joint, lower: f32, upper: f32) -> i32;
    pub fn b2Glue_JointEnableLimit(joint: Joint, enable: i32) -> i32;
    pub fn b2Glue_JointEnableSpring(joint: Joint, enable: i32) -> i32;
    pub fn b2Glue_JointSetSpring(joint: Joint, hertz: f32, damping_ratio: f32) -> i32;
    pub fn b2Glue_JointEnableMotor(joint: Joint, enable: i32) -> i32;
    pub fn b2Glue_JointSetMotor(joint: Joint, speed: f32, max_force: f32) -> i32;
    pub fn b2Glue_JointReaction(joint: Joint, out3: *mut f32) -> i32;
}

/// Wheel joint definition. Body B slides along `axis` (in frame A) and rotates freely.
#[derive(Clone, Copy, Debug, Default)]
pub struct WheelDef {
    pub anchor_a: [f32; 2],
    pub anchor_b: [f32; 2],
    pub axis: [f32; 2],
    /// (hertz, damping ratio). None leaves the spring off.
    pub spring: Option<(f32, f32)>,
    /// (lower, upper) translation limit. None leaves the limit off.
    pub limit: Option<(f32, f32)>,
    /// (speed rad/s, max torque). None leaves the motor off.
    pub motor: Option<(f32, f32)>,
}

/// Reaction on body B from the last step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reaction {
    pub force: [f32; 2],
    pub torque: f32,
}

pub fn create_wheel(world: World, a: Body, b: Body, def: &WheelDef) -> Joint {
    let (spring, hertz, damping) = def.spring.map_or((false, 0.0, 0.0), |(h, d)| (true, h, d));
    let (limit, lower, upper) = def.limit.map_or((false, 0.0, 0.0), |(l, u)| (true, l, u));
    let (motor, speed, torque) = def.motor.map_or((false, 0.0, 0.0), |(s, t)| (true, s, t));
    unsafe {
        b2Glue_CreateWheel(
            world, a, b, def.anchor_a.as_ptr(), def.anchor_b.as_ptr(), def.axis.as_ptr(),
            spring as i32, hertz, damping, limit as i32, lower, upper, motor as i32, speed, torque,
        )
    }
}

/// Mouse joint: anchor `anchor_b` (local to B) is pulled to `target` (world) with a spring of `hertz` and
/// `damping_ratio`, capped at `max_force`. `ground` is the body whose frame holds the target, normally a static body.
pub fn create_mouse(world: World, ground: Body, b: Body, target: [f32; 2], anchor_b: [f32; 2], max_force: f32, hertz: f32, damping_ratio: f32) -> Joint {
    unsafe { b2Glue_CreateMouse(world, ground, b, target.as_ptr(), anchor_b.as_ptr(), max_force, hertz, damping_ratio) }
}

/// Moves a mouse joint's target. Accepts any motor joint, not only ones made by `create_mouse`: the target is the
/// local frame A point of the joint. Returns false for a stale id or a non-motor joint.
#[must_use]
pub fn mouse_set_target(joint: Joint, target: [f32; 2]) -> bool {
    unsafe { b2Glue_MouseSetTarget(joint, target[0], target[1]) != 0 }
}

pub fn create_filter(world: World, a: Body, b: Body) -> Joint {
    unsafe { b2Glue_CreateFilter(world, a, b) }
}

/// Joint kind (`JOINT_*`), or -1 for a stale id.
pub fn joint_kind(joint: Joint) -> i32 {
    unsafe { b2Glue_JointType(joint) }
}

/// Sets the limit of a revolute (angle), prismatic or wheel (translation) joint, or the length range of a distance joint.
/// Returns false for other kinds and for a stale id.
#[must_use]
pub fn set_limits(joint: Joint, lower: f32, upper: f32) -> bool {
    unsafe { b2Glue_JointSetLimits(joint, lower, upper) != 0 }
}

/// Returns false for kinds without a limit and for a stale id.
#[must_use]
pub fn enable_limit(joint: Joint, enable: bool) -> bool {
    unsafe { b2Glue_JointEnableLimit(joint, enable as i32) != 0 }
}

/// Turns the spring on or off. Returns false for kinds without a spring and for a stale id.
#[must_use]
pub fn enable_spring(joint: Joint, enable: bool) -> bool {
    unsafe { b2Glue_JointEnableSpring(joint, enable as i32) != 0 }
}

/// Sets spring stiffness (Hertz) and damping ratio. Returns false for kinds without a spring and for a stale id.
#[must_use]
pub fn set_spring(joint: Joint, hertz: f32, damping_ratio: f32) -> bool {
    unsafe { b2Glue_JointSetSpring(joint, hertz, damping_ratio) != 0 }
}

/// Turns the motor on or off. Returns false for kinds without a motor and for a stale id.
#[must_use]
pub fn enable_motor(joint: Joint, enable: bool) -> bool {
    unsafe { b2Glue_JointEnableMotor(joint, enable as i32) != 0 }
}

/// Sets motor speed and maximum force (torque for revolute and wheel). Returns false for kinds without a motor and for
/// a stale id.
#[must_use]
pub fn set_motor(joint: Joint, speed: f32, max_force: f32) -> bool {
    unsafe { b2Glue_JointSetMotor(joint, speed, max_force) != 0 }
}

/// Reaction forces on body B from the last step. None for an invalid joint.
pub fn reaction(joint: Joint) -> Option<Reaction> {
    let mut out = [0.0f32; 3];
    let ok = unsafe { b2Glue_JointReaction(joint, out.as_mut_ptr()) };
    (ok != 0).then_some(Reaction { force: [out[0], out[1]], torque: out[2] })
}
