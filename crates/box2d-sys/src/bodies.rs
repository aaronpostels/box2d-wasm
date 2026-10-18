//! bodies: safe wrappers over the bodies glue (src/c/box2d_glue_bodies.c). Every body id must be a live body.
use crate::Body;

extern "C" {
    pub fn b2Glue_SetLinearDamping(body: Body, damping: f32);
    pub fn b2Glue_GetLinearDamping(body: Body) -> f32;
    pub fn b2Glue_SetAngularDamping(body: Body, damping: f32);
    pub fn b2Glue_GetAngularDamping(body: Body) -> f32;
    pub fn b2Glue_SetGravityScale(body: Body, scale: f32);
    pub fn b2Glue_GetGravityScale(body: Body) -> f32;
    pub fn b2Glue_SetFixedRotation(body: Body, fixed: bool);
    pub fn b2Glue_GetFixedRotation(body: Body) -> bool;
    pub fn b2Glue_GetMassData(body: Body, out4: *mut f32);
    pub fn b2Glue_SetMassData(body: Body, in4: *const f32);
    pub fn b2Glue_SetBodyUserData(body: Body, value: u32);
    pub fn b2Glue_GetBodyUserData(body: Body) -> u32;
    pub fn b2Glue_EnableBody(body: Body);
    pub fn b2Glue_DisableBody(body: Body);
    pub fn b2Glue_BodyIsEnabled(body: Body) -> bool;
    pub fn b2Glue_ApplyLinearImpulseBatch(ids: *const Body, count: i32, impulses2: *const f32, wake: bool);
    pub fn b2Glue_ApplyAngularImpulseBatch(ids: *const Body, count: i32, angular1: *const f32, wake: bool);
}

/// Mass, center of mass (body local frame) and rotational inertia about the center, as `b2MassData`.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct MassData {
    pub mass: f32,
    pub center: [f32; 2],
    pub rotational_inertia: f32,
}

pub fn set_linear_damping(body: Body, damping: f32) {
    unsafe { b2Glue_SetLinearDamping(body, damping) }
}

pub fn linear_damping(body: Body) -> f32 {
    unsafe { b2Glue_GetLinearDamping(body) }
}

pub fn set_angular_damping(body: Body, damping: f32) {
    unsafe { b2Glue_SetAngularDamping(body, damping) }
}

pub fn angular_damping(body: Body) -> f32 {
    unsafe { b2Glue_GetAngularDamping(body) }
}

/// Scales gravity for this body only: 0 floats it, 1 is normal gravity.
pub fn set_gravity_scale(body: Body, scale: f32) {
    unsafe { b2Glue_SetGravityScale(body, scale) }
}

pub fn gravity_scale(body: Body) -> f32 {
    unsafe { b2Glue_GetGravityScale(body) }
}

/// Locks or unlocks rotation about z. Linear locks are not touched.
pub fn set_fixed_rotation(body: Body, fixed: bool) {
    unsafe { b2Glue_SetFixedRotation(body, fixed) }
}

pub fn is_fixed_rotation(body: Body) -> bool {
    unsafe { b2Glue_GetFixedRotation(body) }
}

pub fn mass_data(body: Body) -> MassData {
    let mut out = [0.0f32; 4];
    unsafe { b2Glue_GetMassData(body, out.as_mut_ptr()) };
    MassData { mass: out[0], center: [out[1], out[2]], rotational_inertia: out[3] }
}

pub fn set_mass_data(body: Body, data: MassData) {
    let input = [data.mass, data.center[0], data.center[1], data.rotational_inertia];
    unsafe { b2Glue_SetMassData(body, input.as_ptr()) }
}

/// Stores a 32-bit value on the body (an entity id, for example). Bodies start at 0.
pub fn set_user_data(body: Body, value: u32) {
    unsafe { b2Glue_SetBodyUserData(body, value) }
}

pub fn user_data(body: Body) -> u32 {
    unsafe { b2Glue_GetBodyUserData(body) }
}

/// A disabled body has no contacts and does not move until it is enabled again.
pub fn enable(body: Body) {
    unsafe { b2Glue_EnableBody(body) }
}

pub fn disable(body: Body) {
    unsafe { b2Glue_DisableBody(body) }
}

pub fn is_enabled(body: Body) -> bool {
    unsafe { b2Glue_BodyIsEnabled(body) }
}

/// Applies one linear impulse per body at its center of mass. `impulses` has the same length as `bodies`.
pub fn apply_linear_impulses(bodies: &[Body], impulses: &[[f32; 2]], wake: bool) {
    assert_eq!(bodies.len(), impulses.len(), "one impulse per body");
    unsafe { b2Glue_ApplyLinearImpulseBatch(bodies.as_ptr(), bodies.len() as i32, impulses.as_ptr() as *const f32, wake) }
}

/// Applies one angular impulse per body. `impulses` has the same length as `bodies`.
pub fn apply_angular_impulses(bodies: &[Body], impulses: &[f32], wake: bool) {
    assert_eq!(bodies.len(), impulses.len(), "one impulse per body");
    unsafe { b2Glue_ApplyAngularImpulseBatch(bodies.as_ptr(), bodies.len() as i32, impulses.as_ptr(), wake) }
}
