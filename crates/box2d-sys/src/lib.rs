//! Box2D as a Rust library. The C engine and a flat glue API are compiled by build.rs; this file declares the glue
//! functions and, on wasm32-unknown-unknown, the few libc functions Box2D calls.
//!
//! Ids are plain integers: a world is a `u32`, bodies, shapes and joints are `u64`. Pointers are `*const f32` of a
//! documented length, so no C struct layout is needed on the Rust side.
#![allow(non_snake_case)]

use core::ffi::c_void;

pub type World = u32;
pub type Body = u64;
pub type Shape = u64;
pub type Joint = u64;

/// Body types, matching `b2BodyType`.
pub const BODY_STATIC: i32 = 0;
pub const BODY_KINEMATIC: i32 = 1;
pub const BODY_DYNAMIC: i32 = 2;

/// Body flags for `b2Glue_CreateBodyEx`.
pub const BODY_ALLOW_SLEEP: i32 = 1;
pub const BODY_BULLET: i32 = 2;
pub const BODY_FIXED_ROTATION: i32 = 4;
pub const BODY_START_ASLEEP: i32 = 8;

/// Shape flags for shape creation.
pub const SHAPE_SENSOR: i32 = 1;
pub const SHAPE_CONTACT_EVENTS: i32 = 2;
pub const SHAPE_HIT_EVENTS: i32 = 4;

/// Starts a task and returns a handle for `TaskFinish`.
pub type TaskEnqueue = unsafe extern "C" fn(task: unsafe extern "C" fn(*mut c_void), ctx: *mut c_void, user: *mut c_void, name: *const core::ffi::c_char) -> *mut c_void;
/// Blocks until the task behind the handle is done.
pub type TaskFinish = unsafe extern "C" fn(handle: *mut c_void, user: *mut c_void);

extern "C" {
    // World
    pub fn b2Glue_CreateWorld(gx: f32, gy: f32, enable_sleep: i32, enable_continuous: i32) -> World;
    pub fn b2Glue_DestroyWorld(world: World);
    pub fn b2Glue_Step(world: World, dt: f32, sub_steps: i32);
    pub fn b2Glue_WorldIsValid(world: World) -> bool;
    pub fn b2Glue_SetGravity(world: World, gx: f32, gy: f32);
    pub fn b2Glue_Configure(world: World, sleeping: bool, continuous: bool);
    pub fn b2Glue_SetContactTuning(world: World, hertz: f32, damping_ratio: f32, push_speed: f32);
    pub fn b2Glue_StateHash(world: World) -> u64;
    pub fn b2Glue_SetTaskSystem(world: World, workers: i32, enqueue: TaskEnqueue, finish: TaskFinish);
    pub fn b2Glue_WorkerCount(world: World) -> i32;

    // Body
    pub fn b2Glue_CreateBody(world: World, kind: i32, px: f32, py: f32, rotation: f32) -> Body;
    /// `pose3` is px py rotation, `vel3` is vx vy angularVelocity.
    pub fn b2Glue_CreateBodyEx(world: World, kind: i32, pose3: *const f32, vel3: *const f32, flags: i32) -> Body;
    pub fn b2Glue_DestroyBody(body: Body);
    pub fn b2Glue_BodyIsValid(body: Body) -> bool;
    pub fn b2Glue_SetTransform(body: Body, px: f32, py: f32, rotation: f32);
    pub fn b2Glue_GetTransform(body: Body, out3: *mut f32);
    pub fn b2Glue_SetTargetTransform(body: Body, pose3: *const f32, dt: f32);
    pub fn b2Glue_SetLinearVelocity(body: Body, vx: f32, vy: f32);
    pub fn b2Glue_GetLinearVelocity(body: Body, out2: *mut f32);
    pub fn b2Glue_SetAngularVelocity(body: Body, omega: f32);
    pub fn b2Glue_GetAngularVelocity(body: Body) -> f32;
    pub fn b2Glue_ApplyForce(body: Body, fx: f32, fy: f32, px: f32, py: f32, wake: bool);
    pub fn b2Glue_ApplyForceToCenter(body: Body, fx: f32, fy: f32, wake: bool);
    pub fn b2Glue_ApplyLinearImpulse(body: Body, ix: f32, iy: f32, px: f32, py: f32, wake: bool);
    pub fn b2Glue_ApplyLinearImpulseToCenter(body: Body, ix: f32, iy: f32, wake: bool);
    pub fn b2Glue_ApplyAngularImpulse(body: Body, impulse: f32, wake: bool);
    pub fn b2Glue_BodyMass(body: Body) -> f32;
    pub fn b2Glue_SetBodyMass(body: Body, mass: f32);
    pub fn b2Glue_BodyIsAwake(body: Body) -> bool;
    pub fn b2Glue_SetSleepThreshold(body: Body, threshold: f32);

    // Batch and events
    /// 3 floats per body: px, py, rotation.
    pub fn b2Glue_GetBatchTransforms(ids: *const u64, count: i32, out: *mut f32);
    /// 6 floats per body: px, py, rotation, vx, vy, angularVelocity.
    pub fn b2Glue_GetBatchState(ids: *const u64, count: i32, out: *mut f32);
    /// Bodies that moved or fell asleep in the last step, 6 floats each. Returns how many moved.
    pub fn b2Glue_MovedBodies(world: World, ids: *mut u64, out: *mut f32, max: i32) -> i32;

    // Shapes
    pub fn b2Glue_AddCircle(body: Body, radius: f32, density: f32, friction: f32, restitution: f32, flags: i32) -> Shape;
    pub fn b2Glue_AddBox(body: Body, hx: f32, hy: f32, density: f32, friction: f32, restitution: f32, flags: i32) -> Shape;
    pub fn b2Glue_AddCapsule(body: Body, half_height: f32, radius: f32, density: f32, friction: f32, restitution: f32, flags: i32) -> Shape;
    pub fn b2Glue_AddSegment(body: Body, x1: f32, y1: f32, x2: f32, y2: f32, density: f32, friction: f32, restitution: f32, flags: i32) -> Shape;
    pub fn b2Glue_AddPolygon(body: Body, points2: *const f32, count: i32, radius: f32, density: f32, friction: f32, restitution: f32, flags: i32) -> Shape;
    pub fn b2Glue_DestroyShape(shape: Shape, update_mass: bool);
    pub fn b2Glue_ShapeIsValid(shape: Shape) -> bool;
    pub fn b2Glue_ShapeBody(shape: Shape) -> Body;

    // Joints
    pub fn b2Glue_CreateWeld(world: World, a: Body, b: Body, anchorA2: *const f32, anchorB2: *const f32, linear_hertz: f32, angular_hertz: f32) -> Joint;
    pub fn b2Glue_CreateRevolute(world: World, a: Body, b: Body, anchorA2: *const f32, anchorB2: *const f32, limited: i32, lower: f32, upper: f32) -> Joint;
    pub fn b2Glue_CreatePrismatic(world: World, a: Body, b: Body, anchorA2: *const f32, anchorB2: *const f32, axis2: *const f32, limited: i32, lower: f32, upper: f32) -> Joint;
    pub fn b2Glue_CreateDistance(world: World, a: Body, b: Body, anchorA2: *const f32, anchorB2: *const f32, length: f32, hertz: f32, damping: f32) -> Joint;
    pub fn b2Glue_CreateMotor(world: World, a: Body, b: Body, max_force: f32, max_torque: f32) -> Joint;
    pub fn b2Glue_DestroyJoint(joint: Joint);
    pub fn b2Glue_JointIsValid(joint: Joint) -> bool;

    // Events
    pub fn b2Glue_ContactBeginEvents(world: World, out: *mut u64, max: i32) -> i32;
    pub fn b2Glue_ContactEndEvents(world: World, out: *mut u64, max: i32) -> i32;
    pub fn b2Glue_SensorBeginEvents(world: World, out: *mut u64, max: i32) -> i32;
    pub fn b2Glue_SensorEndEvents(world: World, out: *mut u64, max: i32) -> i32;

    // Raycast: outResult6 is [fraction, pointX, pointY, normalX, normalY, hit]. Returns hit shape or 0.
    pub fn b2Glue_CastRayClosest(world: World, origin2: *const f32, translation2: *const f32, outResult6: *mut f32) -> Shape;

    // Character mover: moves capsule character by delta2 sliding along contact planes. pos2 is updated. Returns 1 when grounded.
    pub fn b2Glue_MoveCharacter(world: World, self_shape: Shape, pos2: *mut f32, half_height: f32, radius: f32, delta2: *const f32, ground_normal_y: f32) -> i32;

    // Snapshots
    pub fn b2Glue_SnapshotSize(world: World) -> i32;
    pub fn b2Glue_SnapshotTake(world: World, out_buffer: *mut u8, capacity: i32) -> i32;
    pub fn b2Glue_SnapshotRestore(world: World, buffer: *const u8, size: i32) -> bool;
    pub fn b2Glue_SnapshotCreateWorld(buffer: *const u8, size: i32, worker_count: i32) -> World;
}

#[cfg(feature = "rayon")]
static TASKS: [core::sync::atomic::AtomicU32; 10] = [const { core::sync::atomic::AtomicU32::new(0) }; 10];

#[cfg(feature = "rayon")]
struct TaskSlot {
    done: core::sync::atomic::AtomicBool,
    name: [u8; 16],
}

#[cfg(feature = "rayon")]
pub fn task_stats() -> [u32; 10] {
    use core::sync::atomic::Ordering::Relaxed;
    core::array::from_fn(|i| TASKS[i].load(Relaxed))
}

#[cfg(feature = "rayon")]
pub fn use_rayon(world: World, workers: i32) {
    use core::sync::atomic::{AtomicBool, Ordering};

    struct Job(unsafe extern "C" fn(*mut c_void), *mut c_void);
    unsafe impl Send for Job {}

    unsafe extern "C" fn enqueue(task: unsafe extern "C" fn(*mut c_void), ctx: *mut c_void, _user: *mut c_void, _name: *const core::ffi::c_char) -> *mut c_void {
        let mut name = [0u8; 16];
        if !_name.is_null() {
            for (i, slot) in name.iter_mut().enumerate().take(15) {
                let c = unsafe { *(_name as *const u8).add(i) };
                if c == 0 {
                    break;
                }
                *slot = c;
            }
        }
        let slot: &'static TaskSlot = Box::leak(Box::new(TaskSlot { done: AtomicBool::new(false), name }));
        let done = &slot.done;
        let job = Job(task, ctx);
        TASKS[0].fetch_add(1, Ordering::Relaxed);
        rayon::spawn(move || {
            let job = job;
            TASKS[1].fetch_add(1, Ordering::Relaxed);
            unsafe { (job.0)(job.1) };
            TASKS[2].fetch_add(1, Ordering::Relaxed);
            done.store(true, Ordering::Release);
        });
        slot as *const TaskSlot as *mut c_void
    }

    unsafe extern "C" fn finish(handle: *mut c_void, _user: *mut c_void) {
        let slot = &*(handle as *const TaskSlot);
        let done = &slot.done;
        let mut spins = 0u64;
        while !done.load(Ordering::Acquire) {
            core::hint::spin_loop();
            spins += 1;
            if spins > 400_000_000 {
                TASKS[3].store(TASKS[0].load(Ordering::Relaxed), Ordering::Relaxed);
                TASKS[4].store(TASKS[1].load(Ordering::Relaxed), Ordering::Relaxed);
                TASKS[5].store(TASKS[2].load(Ordering::Relaxed), Ordering::Relaxed);
                for k in 0..4 {
                    TASKS[6 + k].store(u32::from_le_bytes([slot.name[4 * k], slot.name[4 * k + 1], slot.name[4 * k + 2], slot.name[4 * k + 3]]), Ordering::Relaxed);
                }
                #[cfg(target_arch = "wasm32")]
                core::arch::wasm32::unreachable();
                #[cfg(not(target_arch = "wasm32"))]
                panic!("box2d task stalled");
            }
        }
        drop(Box::from_raw(handle as *mut TaskSlot));
    }

    unsafe { b2Glue_SetTaskSystem(world, workers, enqueue, finish) };
}

#[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
mod libc {
    use core::alloc::Layout;

    const HEADER: usize = 16;

    unsafe fn alloc_aligned(align: usize, size: usize, zero: bool) -> *mut u8 {
        let align = align.max(HEADER);
        let total = size + align;
        let Ok(layout) = Layout::from_size_align(total, align) else { return core::ptr::null_mut() };
        let base = if zero { std::alloc::alloc_zeroed(layout) } else { std::alloc::alloc(layout) };
        if base.is_null() {
            return base;
        }
        let user = base.add(align);
        (user.sub(16) as *mut usize).write(total);
        (user.sub(8) as *mut usize).write(align);
        user
    }

    #[no_mangle]
    pub unsafe extern "C" fn malloc(size: usize) -> *mut u8 {
        alloc_aligned(HEADER, size, false)
    }
    #[no_mangle]
    pub unsafe extern "C" fn aligned_alloc(align: usize, size: usize) -> *mut u8 {
        alloc_aligned(align, size, false)
    }
    #[no_mangle]
    pub unsafe extern "C" fn calloc(n: usize, m: usize) -> *mut u8 {
        match n.checked_mul(m) {
            Some(size) => alloc_aligned(HEADER, size, true),
            None => core::ptr::null_mut(),
        }
    }
    #[no_mangle]
    pub unsafe extern "C" fn free(p: *mut u8) {
        if p.is_null() {
            return;
        }
        let total = (p.sub(16) as *const usize).read();
        let align = (p.sub(8) as *const usize).read();
        std::alloc::dealloc(p.sub(align), Layout::from_size_align_unchecked(total, align));
    }
    #[no_mangle]
    pub unsafe extern "C" fn realloc(p: *mut u8, size: usize) -> *mut u8 {
        if p.is_null() {
            return malloc(size);
        }
        let total = (p.sub(16) as *const usize).read();
        let align = (p.sub(8) as *const usize).read();
        let old = total - align;
        let n = malloc(size);
        if !n.is_null() {
            core::ptr::copy_nonoverlapping(p, n, old.min(size));
            free(p);
        }
        n
    }
    #[no_mangle]
    pub unsafe extern "C" fn strlen(s: *const u8) -> usize {
        let mut n = 0;
        while *s.add(n) != 0 {
            n += 1;
        }
        n
    }
}
