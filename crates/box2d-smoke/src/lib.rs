//! Runs Box2D inside a plain wasm32-unknown-unknown module: drop boxes/balls on a floor, step,
//! test contacts, snapshot, restore, and raycast. `run` returns 0 when everything holds,
//! or a code naming the failed check.
use box2d_sys::joints as jt;
use box2d_sys::*;
use box2d_sys::{bodies as bd, filters as fl};

fn build() -> (World, Vec<Body>) {
    unsafe {
        let w = b2Glue_CreateWorld(0.0, -10.0, 0, 0);
        let vel = [0.0f32; 3];
        let floor_pose = [0.0f32, -1.0, 0.0];
        let ground = b2Glue_CreateBodyEx(w, BODY_STATIC, floor_pose.as_ptr(), vel.as_ptr(), 0);
        b2Glue_AddBox(ground, 50.0, 1.0, 1.0, 0.5, 0.0, SHAPE_CONTACT_EVENTS);

        let mut ids = Vec::new();
        for i in 0..100 {
            let (x, y) = ((i % 10) as f32 * 2.1 - 10.0, 2.0 + (i / 10) as f32 * 2.1);
            let pose = [x, y, 0.0f32];
            let b = b2Glue_CreateBodyEx(w, BODY_DYNAMIC, pose.as_ptr(), vel.as_ptr(), BODY_ALLOW_SLEEP);
            if i % 2 == 0 {
                b2Glue_AddCircle(b, 0.9, 1.0, 0.5, 0.0, SHAPE_CONTACT_EVENTS);
            } else {
                b2Glue_AddBox(b, 0.9, 0.9, 1.0, 0.5, 0.0, SHAPE_CONTACT_EVENTS);
            }
            ids.push(b);
        }
        (w, ids)
    }
}

fn state(ids: &[Body]) -> Vec<f32> {
    let mut out = vec![0.0f32; ids.len() * 6];
    unsafe { b2Glue_GetBatchState(ids.as_ptr(), ids.len() as i32, out.as_mut_ptr()) };
    out
}

// Shapes section. Each check returns its own code (20s chain, 30s one-sided chain, 40s properties, 50s rolling
// resistance, 60s tangent speed) so a failure names itself. 0 means pass.
use box2d_sys::shapes as sh;

const ZERO3: [f32; 3] = [0.0; 3];

fn new_world() -> World {
    unsafe { b2Glue_CreateWorld(0.0, -10.0, 0, 0) }
}

fn body_at(w: World, kind: i32, pose: [f32; 3], vel: [f32; 3]) -> Body {
    unsafe { b2Glue_CreateBodyEx(w, kind, pose.as_ptr(), vel.as_ptr(), 0) }
}

// Static floor whose top face is at y = 0.
fn floor(w: World) -> Shape {
    let ground = body_at(w, BODY_STATIC, [0.0, -1.0, 0.0], ZERO3);
    unsafe { b2Glue_AddBox(ground, 50.0, 1.0, 1.0, 0.5, 0.0, 0) }
}

fn step(w: World, n: i32) {
    for _ in 0..n {
        unsafe { b2Glue_Step(w, 1.0 / 60.0, 4) };
    }
}

fn position(b: Body) -> [f32; 3] {
    let mut t = [0.0f32; 3];
    unsafe { b2Glue_GetTransform(b, t.as_mut_ptr()) };
    t
}

fn velocity(b: Body) -> [f32; 2] {
    let mut v = [0.0f32; 2];
    unsafe { b2Glue_GetLinearVelocity(b, v.as_mut_ptr()) };
    v
}

fn rounded_polygon() -> i32 {
    unsafe {
        let w = new_world();
        floor(w);
        let square = [-1.0f32, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0];
        let body = body_at(w, BODY_DYNAMIC, [0.0, 4.0, 0.0], ZERO3);
        let poly = b2Glue_AddPolygon(body, square.as_ptr(), 4, 0.25, 1.0, 0.5, 0.0, 0);
        if poly == 0 || (sh::radius(poly) - 0.25).abs() > 1e-6 {
            return 10;
        }
        step(w, 240);
        // The 2 x 2 core plus a 0.25 radius all round rests with its center at 1.25. A sharp box would sit at 1.0.
        let y = position(body)[1];
        if !(y > 1.1 && y < 1.4) {
            return 11;
        }
        b2Glue_DestroyWorld(w);
    }
    0
}

fn chain_loop() -> i32 {
    unsafe {
        let w = new_world();
        let frame = body_at(w, BODY_STATIC, ZERO3, ZERO3);
        // Clockwise, so the solid side of every segment (its right-hand side) faces the inside.
        let square = [[-5.0f32, -5.0], [-5.0, 5.0], [5.0, 5.0], [5.0, -5.0]];
        let Some(chain) = sh::add_chain(frame, &square, true, 0.6, 0.0, 0.0, 0.0) else {
            return 20;
        };
        if !sh::chain_is_valid(chain) || sh::chain_segment_count(chain) != 4 {
            return 21;
        }
        let ball = body_at(w, BODY_DYNAMIC, [0.0, 3.0, 0.0], ZERO3);
        b2Glue_AddCircle(ball, 0.5, 1.0, 0.5, 0.0, 0);
        step(w, 300);
        // Resting on the bottom edge (y = -5) with radius 0.5: the ball stayed inside the loop.
        let p = position(ball);
        if (p[1] + 4.5).abs() > 0.1 || p[0].abs() > 4.5 {
            return 22;
        }
        b2Glue_DestroyWorld(w);
    }
    0
}

fn chain_one_sided() -> i32 {
    // Right to left, so the right-hand side of each segment (the solid side) is the top.
    let line = [[5.0f32, 0.0], [0.0, 0.0], [-5.0, 0.0]];
    unsafe {
        // From above: the top face is solid, the ball lands on it.
        let w = new_world();
        let frame = body_at(w, BODY_STATIC, ZERO3, ZERO3);
        let Some(chain) = sh::add_chain(frame, &line, false, 0.6, 0.0, 0.0, 0.0) else {
            return 30;
        };
        if sh::chain_segment_count(chain) != 2 {
            return 31;
        }
        let ball = body_at(w, BODY_DYNAMIC, [2.0, 3.0, 0.0], ZERO3);
        b2Glue_AddCircle(ball, 0.5, 1.0, 0.5, 0.0, 0);
        step(w, 180);
        if (position(ball)[1] - 0.5).abs() > 0.1 {
            return 32;
        }
        b2Glue_DestroyWorld(w);

        // From below: the underside is open, so a ball moving up passes through and rises above y = 0.
        let w = new_world();
        let frame = body_at(w, BODY_STATIC, ZERO3, ZERO3);
        sh::add_chain(frame, &line, false, 0.6, 0.0, 0.0, 0.0);
        let ball = body_at(w, BODY_DYNAMIC, [-2.0, -3.0, 0.0], [0.0, 8.0, 0.0]);
        b2Glue_AddCircle(ball, 0.5, 1.0, 0.5, 0.0, 0);
        let mut highest = f32::MIN;
        for _ in 0..60 {
            step(w, 1);
            highest = highest.max(position(ball)[1]);
        }
        if highest <= 0.0 {
            return 33; // blocked from below
        }
        b2Glue_DestroyWorld(w);
    }
    0
}

fn properties() -> i32 {
    unsafe {
        let w = new_world();
        let body = body_at(w, BODY_DYNAMIC, [0.0, 10.0, 0.0], ZERO3);
        let shape = b2Glue_AddBox(body, 0.5, 0.5, 1.0, 0.5, 0.2, 0);
        if (sh::friction(shape) - 0.5).abs() > 1e-6 {
            return 40;
        }
        if (sh::restitution(shape) - 0.2).abs() > 1e-6 {
            return 41;
        }
        if (sh::density(shape) - 1.0).abs() > 1e-6 {
            return 42;
        }
        // Unit box at density 1: mass 1.
        if (b2Glue_BodyMass(body) - 1.0).abs() > 1e-4 {
            return 43;
        }
        sh::set_friction(shape, 0.9);
        sh::set_restitution(shape, 0.7);
        sh::set_density(shape, 3.0, true);
        if (sh::friction(shape) - 0.9).abs() > 1e-6 {
            return 44;
        }
        if (sh::restitution(shape) - 0.7).abs() > 1e-6 {
            return 45;
        }
        if (sh::density(shape) - 3.0).abs() > 1e-6 {
            return 46;
        }
        // Density change with the mass update: mass 3 now.
        if (b2Glue_BodyMass(body) - 3.0).abs() > 1e-4 {
            return 47;
        }
        if sh::is_sensor(shape) {
            return 48;
        }
        let sensor = b2Glue_AddBox(body, 0.1, 0.1, 1.0, 0.5, 0.0, SHAPE_SENSOR);
        if !sh::is_sensor(sensor) {
            return 49;
        }
        b2Glue_DestroyWorld(w);
    }
    0
}

// Ball rolling right on the floor at 5 m/s (angular velocity -v / r). Returns its speed after 2 s.
fn roll_speed(rolling: f32) -> Result<f32, i32> {
    unsafe {
        let w = new_world();
        floor(w);
        let ball = body_at(w, BODY_DYNAMIC, [0.0, 0.5, 0.0], [5.0, 0.0, -10.0]);
        let shape = b2Glue_AddCircle(ball, 0.5, 1.0, 0.6, 0.0, 0);
        sh::set_rolling_resistance(shape, rolling);
        if sh::rolling_resistance(shape) != rolling {
            return Err(50);
        }
        step(w, 120);
        let v = velocity(ball)[0];
        b2Glue_DestroyWorld(w);
        Ok(v)
    }
}

fn rolling_resistance() -> i32 {
    let free = match roll_speed(0.0) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let braked = match roll_speed(1.0) {
        Ok(v) => v,
        Err(code) => return code,
    };
    if free < 4.0 {
        return 51; // without resistance the ball should keep rolling
    }
    if braked > free - 0.5 {
        return 52; // resistance must slow it clearly
    }
    0
}

// Box on a floor with the given surface tangent speed. Returns how far it travels along x in 3 s.
fn belt_travel(tangent: f32) -> Result<f32, i32> {
    unsafe {
        let w = new_world();
        let ground = floor(w);
        sh::set_tangent_speed(ground, tangent);
        if sh::tangent_speed(ground) != tangent {
            return Err(60);
        }
        let body = body_at(w, BODY_DYNAMIC, [0.0, 1.0, 0.0], ZERO3);
        b2Glue_AddBox(body, 0.5, 0.5, 1.0, 0.6, 0.0, 0);
        step(w, 180);
        let x = position(body)[0];
        b2Glue_DestroyWorld(w);
        Ok(x)
    }
}

fn tangent_speed() -> i32 {
    match belt_travel(0.0) {
        Ok(x) if x.abs() > 0.1 => return 61, // no belt: the box stays put
        Err(code) => return code,
        _ => {}
    }
    match belt_travel(3.0) {
        Ok(x) if x.abs() < 1.0 => 62, // belt at 3 m/s must carry it
        Err(code) => code,
        _ => 0,
    }
}

fn shapes() -> i32 {
    for check in [rounded_polygon, chain_loop, chain_one_sided, properties, rolling_resistance, tangent_speed] {
        let code = check();
        if code != 0 {
            return code;
        }
    }
    0
}

/// Runs only the shapes checks, so test/shapes.mjs can call them alone.
#[no_mangle]
pub extern "C" fn run_shapes() -> i32 {
    shapes()
}

// Joint checks: each returns 0 when it passes, or the code of the check that failed (10 to 19; 39 is the revolute
// limit overshoot, so the kind-mismatch 10 and the overshoot are told apart).
fn joints() -> i32 {
    for check in [joint_revolute_limit, joint_revolute_motor, joint_prismatic_limit, joint_prismatic_spring, joint_distance_spring, joint_wheel_limit, joint_wheel_motor, joint_mouse, joint_filter, joint_reaction] {
        let code = check();
        if code != 0 {
            return code;
        }
    }
    0
}

#[no_mangle]
pub extern "C" fn run_joints() -> i32 {
    joints()
}

fn joint_world(gravity_y: f32) -> World {
    unsafe { b2Glue_CreateWorld(0.0, gravity_y, 0, 0) }
}

fn joint_static(w: World, x: f32, y: f32) -> Body {
    let pose = [x, y, 0.0f32];
    let vel = [0.0f32; 3];
    unsafe { b2Glue_CreateBodyEx(w, BODY_STATIC, pose.as_ptr(), vel.as_ptr(), 0) }
}

// Dynamic box with density 1, so its mass is 4 * half * half.
fn joint_box(w: World, x: f32, y: f32, half: f32) -> Body {
    let pose = [x, y, 0.0f32];
    let vel = [0.0f32; 3];
    unsafe {
        let b = b2Glue_CreateBodyEx(w, BODY_DYNAMIC, pose.as_ptr(), vel.as_ptr(), 0);
        b2Glue_AddBox(b, half, half, 1.0, 0.5, 0.0, 0);
        b
    }
}

fn joint_steps(w: World, n: usize) {
    for _ in 0..n {
        unsafe { b2Glue_Step(w, 1.0 / 60.0, 4) };
    }
}

fn joint_pose(b: Body) -> [f32; 3] {
    let mut out = [0.0f32; 3];
    unsafe { b2Glue_GetTransform(b, out.as_mut_ptr()) };
    out
}

// Revolute limit: a pendulum released at 1 rad must stay inside its +-0.3 rad limit. Without the limit it swings past it.
fn joint_revolute_limit() -> i32 {
    let w = joint_world(-10.0);
    let pivot = joint_static(w, 0.0, 5.0);
    let bob = joint_box(w, 0.0, 4.0, 0.5);
    let (anchor_a, anchor_b) = ([0.0f32, 0.0], [0.0f32, 1.0]);
    let rev = unsafe { b2Glue_CreateRevolute(w, pivot, bob, anchor_a.as_ptr(), anchor_b.as_ptr(), 1, -0.3, 0.3) };
    let theta = 1.0f32;
    unsafe { b2Glue_SetTransform(bob, theta.sin(), 5.0 - theta.cos(), theta) };
    if jt::joint_kind(rev) != jt::JOINT_REVOLUTE {
        return 10;
    }
    // The solver pulls the bob from 1 rad back inside the limit over the first steps, so judge after one second.
    joint_steps(w, 60);
    for _ in 0..180 {
        joint_steps(w, 1);
        if joint_pose(bob)[2].abs() > 0.33 {
            return 39;
        }
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Revolute motor: with no gravity a free wheel must reach its set speed of 2 rad/s.
fn joint_revolute_motor() -> i32 {
    let w = joint_world(0.0);
    let hub = joint_static(w, 0.0, 0.0);
    let wheel = joint_box(w, 3.0, 0.0, 0.5);
    let (anchor_a, anchor_b) = ([0.0f32, 0.0], [-3.0f32, 0.0]);
    let rev = unsafe { b2Glue_CreateRevolute(w, hub, wheel, anchor_a.as_ptr(), anchor_b.as_ptr(), 0, 0.0, 0.0) };
    if !jt::enable_motor(rev, true) || !jt::set_motor(rev, 2.0, 1000.0) {
        return 11;
    }
    joint_steps(w, 60);
    let omega = unsafe { b2Glue_GetAngularVelocity(wheel) };
    if (omega - 2.0).abs() > 0.05 {
        return 11;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Prismatic limit: a slider kicked along x must stop at its +-0.5 limit.
fn joint_prismatic_limit() -> i32 {
    let w = joint_world(0.0);
    let rail = joint_static(w, 0.0, 0.0);
    let slider = joint_box(w, 0.0, 0.0, 0.5);
    let (anchor, axis) = ([0.0f32, 0.0], [1.0f32, 0.0]);
    let pr = unsafe { b2Glue_CreatePrismatic(w, rail, slider, anchor.as_ptr(), anchor.as_ptr(), axis.as_ptr(), 0, 0.0, 0.0) };
    if !jt::set_limits(pr, -0.5, 0.5) || !jt::enable_limit(pr, true) {
        return 12;
    }
    unsafe { b2Glue_ApplyLinearImpulseToCenter(slider, 5.0, 0.0, true) };
    joint_steps(w, 120);
    let x = joint_pose(slider)[0];
    if !(0.45..=0.52).contains(&x) {
        return 12;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Prismatic spring: a slider hanging on a vertical axis sags by g / w^2 = 0.063 m at 2 Hz (damping 0.7).
fn joint_prismatic_spring() -> i32 {
    let w = joint_world(-10.0);
    let rail = joint_static(w, 0.0, 0.0);
    let slider = joint_box(w, 0.0, 0.0, 0.5);
    let (anchor, axis) = ([0.0f32, 0.0], [0.0f32, 1.0]);
    let pr = unsafe { b2Glue_CreatePrismatic(w, rail, slider, anchor.as_ptr(), anchor.as_ptr(), axis.as_ptr(), 0, 0.0, 0.0) };
    if !jt::set_spring(pr, 2.0, 0.7) || !jt::enable_spring(pr, true) {
        return 13;
    }
    joint_steps(w, 480);
    let y = joint_pose(slider)[1];
    if !(-0.08..-0.05).contains(&y) {
        return 13;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Distance: a rigid rod (spring off) holds length 2. Turning the spring on at 2 Hz lets the weight sag by g / w^2.
fn joint_distance_spring() -> i32 {
    let w = joint_world(-10.0);
    let ceiling = joint_static(w, 0.0, 10.0);
    let weight = joint_box(w, 0.0, 8.0, 0.5);
    let anchor = [0.0f32, 0.0];
    let d = unsafe { b2Glue_CreateDistance(w, ceiling, weight, anchor.as_ptr(), anchor.as_ptr(), 2.0, 0.0, 0.0) };
    joint_steps(w, 240);
    if (joint_pose(weight)[1] - 8.0).abs() > 0.05 {
        return 14;
    }
    if !jt::set_spring(d, 2.0, 0.7) || !jt::enable_spring(d, true) {
        return 14;
    }
    joint_steps(w, 480);
    let y = joint_pose(weight)[1];
    if !(7.85..7.99).contains(&y) {
        return 14;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Wheel limit: the wheel slides along its vertical axis and must stop at +-0.5.
fn joint_wheel_limit() -> i32 {
    let w = joint_world(0.0);
    let base = joint_static(w, 0.0, 0.0);
    let slider = joint_box(w, 0.0, 0.0, 0.5);
    let def = jt::WheelDef { axis: [0.0, 1.0], limit: Some((-0.5, 0.5)), ..Default::default() };
    let wheel = jt::create_wheel(w, base, slider, &def);
    if jt::joint_kind(wheel) != jt::JOINT_WHEEL {
        return 15;
    }
    unsafe { b2Glue_ApplyLinearImpulseToCenter(slider, 0.0, 5.0, true) };
    joint_steps(w, 120);
    let y = joint_pose(slider)[1];
    if !(0.45..=0.52).contains(&y) {
        return 15;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Wheel motor: the wheel's rotational motor must reach 3 rad/s with no gravity.
fn joint_wheel_motor() -> i32 {
    let w = joint_world(0.0);
    let base = joint_static(w, 0.0, 0.0);
    let wheel_body = joint_box(w, 0.0, 0.0, 0.5);
    let def = jt::WheelDef { axis: [1.0, 0.0], motor: Some((3.0, 1000.0)), ..Default::default() };
    let wheel = jt::create_wheel(w, base, wheel_body, &def);
    if jt::joint_kind(wheel) != jt::JOINT_WHEEL {
        return 16;
    }
    joint_steps(w, 60);
    let omega = unsafe { b2Glue_GetAngularVelocity(wheel_body) };
    if (omega - 3.0).abs() > 0.1 {
        return 16;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Mouse: the pulled point follows the target, the target can move, and the setter refuses non-mouse joints.
fn joint_mouse() -> i32 {
    let w = joint_world(0.0);
    let ground = joint_static(w, 0.0, 0.0);
    let body = joint_box(w, 0.0, 0.0, 0.5);
    let mouse = jt::create_mouse(w, ground, body, [2.0, 0.0], [0.0, 0.0], 1000.0, 4.0, 0.7);
    if jt::joint_kind(mouse) != jt::JOINT_MOTOR {
        return 17;
    }
    joint_steps(w, 180);
    if (joint_pose(body)[0] - 2.0).abs() > 0.2 {
        return 17;
    }
    if !jt::mouse_set_target(mouse, [-2.0, 0.0]) {
        return 17;
    }
    joint_steps(w, 180);
    if (joint_pose(body)[0] + 2.0).abs() > 0.2 {
        return 17;
    }
    let filter = jt::create_filter(w, ground, body);
    if jt::mouse_set_target(filter, [0.0, 0.0]) {
        return 17;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Filter: a box joined to the floor by a filter joint falls through it. Without the joint it rests on the floor.
fn joint_filter() -> i32 {
    let w = joint_world(-10.0);
    let floor = joint_static(w, 0.0, 0.0);
    unsafe { b2Glue_AddBox(floor, 10.0, 0.5, 1.0, 0.5, 0.0, 0) };
    let falling = joint_box(w, 0.0, 1.0, 0.5);
    let f = jt::create_filter(w, floor, falling);
    if jt::joint_kind(f) != jt::JOINT_FILTER {
        return 18;
    }
    if jt::set_limits(f, 0.0, 1.0) || jt::enable_spring(f, true) || jt::enable_motor(f, true) {
        return 18; // a filter joint has no limit, spring or motor
    }
    joint_steps(w, 120);
    if joint_pose(falling)[1] > -1.0 {
        return 18;
    }
    unsafe { b2Glue_DestroyWorld(w) };

    let w = joint_world(-10.0);
    let floor = joint_static(w, 0.0, 0.0);
    unsafe { b2Glue_AddBox(floor, 10.0, 0.5, 1.0, 0.5, 0.0, 0) };
    let resting = joint_box(w, 0.0, 1.0, 0.5);
    joint_steps(w, 120);
    if (joint_pose(resting)[1] - 1.0).abs() > 0.05 {
        return 18;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Reaction: a box hanging from a rigid rod carries its weight (mass 1, g 10), so the rod pulls with 10 N and no torque.
fn joint_reaction() -> i32 {
    let w = joint_world(-10.0);
    let ceiling = joint_static(w, 0.0, 2.0);
    let hanging = joint_box(w, 0.0, 0.0, 0.5);
    let anchor = [0.0f32, 0.0];
    let rod = unsafe { b2Glue_CreateDistance(w, ceiling, hanging, anchor.as_ptr(), anchor.as_ptr(), 2.0, 0.0, 0.0) };
    joint_steps(w, 240);
    let Some(r) = jt::reaction(rod) else {
        return 19;
    };
    let magnitude = (r.force[0] * r.force[0] + r.force[1] * r.force[1]).sqrt();
    if (magnitude - 10.0).abs() > 0.5 || r.torque.abs() > 0.5 {
        return 19;
    }
    unsafe { b2Glue_DestroyWorld(w) };
    0
}

// Codes 20..=38 name the failed check. Scene: static box A at origin, dynamic circle B at x=5, static box C at x=10,
// static box D at y=10. Gravity is zero so nothing moves.
fn queries() -> i32 {
    use box2d_sys::queries::*;
    unsafe {
        let w = b2Glue_CreateWorld(0.0, 0.0, 0, 0);
        let zero = [0.0f32; 3];
        let at = |x: f32, y: f32, kind: i32| {
            let pose = [x, y, 0.0f32];
            b2Glue_CreateBodyEx(w, kind, pose.as_ptr(), zero.as_ptr(), 0)
        };
        let a = b2Glue_AddBox(at(0.0, 0.0, BODY_STATIC), 1.0, 1.0, 1.0, 0.5, 0.0, 0);
        let b_body = at(5.0, 0.0, BODY_DYNAMIC);
        let b = b2Glue_AddCircle(b_body, 0.5, 1.0, 0.5, 0.0, 0);
        let c = b2Glue_AddBox(at(10.0, 0.0, BODY_STATIC), 1.0, 1.0, 1.0, 0.5, 0.0, 0);
        b2Glue_AddBox(at(0.0, 10.0, BODY_STATIC), 1.0, 1.0, 1.0, 0.5, 0.0, 0);

        // Overlap AABB: a small box hits only A; a wide strip hits A and B, not C.
        let mut ids = [0u64; 8];
        if overlap_aabb(w, [-2.0, -2.0], [2.0, 2.0], &mut ids) != 1 || ids[0] != a {
            return 20;
        }
        if overlap_aabb(w, [-6.0, -1.0], [6.0, 1.0], &mut ids) != 2 || !ids[..2].contains(&a) || !ids[..2].contains(&b) {
            return 21;
        }

        // Overlap circle: inside B only, then clear of everything.
        if overlap_circle(w, [5.0, 0.0], 0.1, &mut ids) != 1 || ids[0] != b {
            return 22;
        }
        if overlap_circle(w, [0.0, 3.0], 0.5, &mut ids) != 0 {
            return 23;
        }

        // Overlap polygon: a triangle around B's centre, then invalid point counts.
        let tri = [[4.0f32, -1.0], [6.0, -1.0], [5.0, 1.0]];
        if overlap_polygon(w, &tri, 0.0, &mut ids) != Some(1) || ids[0] != b {
            return 24;
        }
        if overlap_polygon(w, &[], 0.0, &mut ids).is_some() || overlap_polygon(w, &[[0.0f32, 0.0]; 9], 0.0, &mut ids).is_some() {
            return 25;
        }

        // Ray, all hits: from x=-5 towards +x it crosses A (t=0.2), B (t=0.475) and C (t=0.7). The glue sorts them,
        // nearest first.
        let mut hits = [QueryHit::default(); 4];
        if cast_ray_all(w, [-5.0, 0.0], [20.0, 0.0], &mut hits) != 3 {
            return 26;
        }
        if hits[0].shape != a || (hits[0].fraction - 0.2).abs() > 1e-4 || (hits[0].normal[0] + 1.0).abs() > 1e-4 {
            return 27;
        }
        if hits[1].shape != b || (hits[1].fraction - 0.475).abs() > 1e-3 {
            return 28;
        }
        if hits[2].shape != c || (hits[2].fraction - 0.7).abs() > 1e-3 {
            return 29;
        }
        // A buffer shorter than the hit count still reports the full count, and keeps the two nearest hits in order.
        let mut two = [QueryHit::default(); 2];
        if cast_ray_all(w, [-5.0, 0.0], [20.0, 0.0], &mut two) != 3 {
            return 30;
        }
        if two[0].shape != a || two[1].shape != b {
            return 30;
        }

        // Shape cast: a radius 0.5 circle moved from x=-5 meets A's left face at t=0.175.
        let circle = [[0.0f32, 0.0]];
        let Some(h) = cast_shape_closest(w, &circle, 0.5, [-5.0, 0.0], [20.0, 0.0]) else {
            return 31;
        };
        if h.shape != a || (h.fraction - 0.175).abs() > 1e-3 || (h.normal[0] + 1.0).abs() > 1e-3 {
            return 32;
        }
        // Passing above A, B and C at y=5 misses. An empty proxy is rejected.
        if cast_shape_closest(w, &circle, 0.5, [-5.0, 5.0], [20.0, 0.0]).is_some() {
            return 33;
        }
        if cast_shape_closest(w, &[], 0.5, [-5.0, 0.0], [20.0, 0.0]).is_some() {
            return 34;
        }
        b2Glue_DestroyWorld(w);

        // Explosion at the origin, radius 5, falloff 2. Circles at surface distance 1.5 (full impulse), 5.5 (scale
        // (5 + 2 - 5.5) / 2 = 0.75) and 7.5 (outside, nothing). Equal masses, so velocity ratios equal impulse ratios.
        let w2 = b2Glue_CreateWorld(0.0, 0.0, 0, 0);
        let circle_at = |x: f32| {
            let pose = [x, 0.0f32, 0.0];
            let body = b2Glue_CreateBodyEx(w2, BODY_DYNAMIC, pose.as_ptr(), zero.as_ptr(), 0);
            b2Glue_AddCircle(body, 0.5, 1.0, 0.5, 0.0, 0);
            body
        };
        let near = circle_at(2.0);
        let mid = circle_at(6.0);
        let far = circle_at(8.0);
        explode(w2, [0.0, 0.0], 5.0, 2.0, 1.0, u64::MAX);
        let velocity = |body: u64| {
            let mut v = [0.0f32; 2];
            b2Glue_GetLinearVelocity(body, v.as_mut_ptr());
            v
        };
        let (vn, vm, vf) = (velocity(near), velocity(mid), velocity(far));
        if !(vn[0] > 0.1) || vn[1].abs() > 1e-5 {
            return 35; // inside the radius: pushed straight outwards
        }
        if (vm[0] / vn[0] - 0.75).abs() > 1e-3 || vm[1].abs() > 1e-5 {
            return 36; // inside the falloff band: 0.75 of the full impulse
        }
        if vf != [0.0, 0.0] {
            return 37; // beyond radius + falloff: untouched
        }
        b2Glue_DestroyWorld(w2);
    }
    0
}

fn dynamic_body(w: World, x: f32, y: f32) -> Body {
    let pose = [x, y, 0.0f32];
    let vel = [0.0f32; 3];
    unsafe { b2Glue_CreateBodyEx(w, BODY_DYNAMIC, pose.as_ptr(), vel.as_ptr(), 0) }
}

fn static_body(w: World, x: f32, y: f32) -> Body {
    let pose = [x, y, 0.0f32];
    let vel = [0.0f32; 3];
    unsafe { b2Glue_CreateBodyEx(w, BODY_STATIC, pose.as_ptr(), vel.as_ptr(), 0) }
}

fn linear_velocity(body: Body) -> [f32; 2] {
    let mut v = [0.0f32; 2];
    unsafe { b2Glue_GetLinearVelocity(body, v.as_mut_ptr()) };
    v
}

fn bodies() -> i32 {
    unsafe {
        // Linear damping: a damped body slows down, an undamped one keeps its speed (no gravity).
        let w = b2Glue_CreateWorld(0.0, 0.0, 0, 0);
        let free = dynamic_body(w, 0.0, 0.0);
        let damped = dynamic_body(w, 0.0, 5.0);
        b2Glue_AddCircle(free, 0.5, 1.0, 0.5, 0.0, 0);
        b2Glue_AddCircle(damped, 0.5, 1.0, 0.5, 0.0, 0);
        bd::set_linear_damping(damped, 2.0);
        if bd::linear_damping(damped) != 2.0 || bd::linear_damping(free) != 0.0 {
            return 10;
        }
        b2Glue_SetLinearVelocity(free, 10.0, 0.0);
        b2Glue_SetLinearVelocity(damped, 10.0, 0.0);
        for _ in 0..60 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
        }
        if (linear_velocity(free)[0] - 10.0).abs() > 1e-3 {
            return 11;
        }
        // Box2D divides v by (1 + dt * c) each step: after one second with c = 2 that is about 10 * e^-2.
        let dv = linear_velocity(damped)[0];
        if !(dv > 1.0 && dv < 2.0) {
            return 12;
        }
        b2Glue_DestroyWorld(w);

        // Gravity scale: 1 falls at g, 0 floats.
        let w = b2Glue_CreateWorld(0.0, -10.0, 0, 0);
        let normal = dynamic_body(w, 0.0, 100.0);
        let floating = dynamic_body(w, 5.0, 100.0);
        b2Glue_AddCircle(normal, 0.5, 1.0, 0.5, 0.0, 0);
        b2Glue_AddCircle(floating, 0.5, 1.0, 0.5, 0.0, 0);
        bd::set_gravity_scale(floating, 0.0);
        if bd::gravity_scale(floating) != 0.0 || bd::gravity_scale(normal) != 1.0 {
            return 20;
        }
        for _ in 0..30 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
        }
        // 0.5 s at g = 10 gives -5 m/s.
        if (linear_velocity(normal)[1] + 5.0).abs() > 0.1 {
            return 21;
        }
        if linear_velocity(floating)[1].abs() > 1e-6 {
            return 22;
        }
        b2Glue_DestroyWorld(w);

        // Fixed rotation: the locked body keeps angle and spin at zero, its linear motion is unaffected.
        let w = b2Glue_CreateWorld(0.0, 0.0, 0, 0);
        let spin = dynamic_body(w, 0.0, 0.0);
        let locked = dynamic_body(w, 10.0, 0.0);
        b2Glue_AddBox(spin, 0.5, 0.5, 1.0, 0.5, 0.0, 0);
        b2Glue_AddBox(locked, 0.5, 0.5, 1.0, 0.5, 0.0, 0);
        b2Glue_SetAngularVelocity(spin, 5.0);
        b2Glue_SetAngularVelocity(locked, 5.0);
        b2Glue_SetLinearVelocity(locked, 3.0, 0.0);
        bd::set_fixed_rotation(locked, true);
        if !bd::is_fixed_rotation(locked) || bd::is_fixed_rotation(spin) {
            return 30;
        }
        for _ in 0..60 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
        }
        if b2Glue_GetAngularVelocity(spin) < 4.5 {
            return 31;
        }
        if b2Glue_GetAngularVelocity(locked).abs() > 1e-6 || position(locked)[2].abs() > 1e-6 {
            return 32;
        }
        if position(locked)[0] < 12.5 {
            return 33; // should have moved about 3 m in x
        }
        bd::set_fixed_rotation(locked, false);
        if bd::is_fixed_rotation(locked) {
            return 34;
        }
        b2Glue_SetAngularVelocity(locked, 2.0);
        b2Glue_Step(w, 1.0 / 60.0, 4);
        if b2Glue_GetAngularVelocity(locked) < 1.5 {
            return 35;
        }
        b2Glue_DestroyWorld(w);

        // Mass data: a 1 m box of density 2 has mass 2 and inertia m (w^2 + h^2) / 12 = 1/3.
        let w = b2Glue_CreateWorld(0.0, -10.0, 0, 0);
        let b = dynamic_body(w, 0.0, 0.0);
        b2Glue_AddBox(b, 0.5, 0.5, 2.0, 0.5, 0.0, 0);
        let m = bd::mass_data(b);
        if (m.mass - 2.0).abs() > 1e-4 || (m.rotational_inertia - 1.0 / 3.0).abs() > 1e-4 {
            return 40;
        }
        if m.center[0].abs() > 1e-6 || m.center[1].abs() > 1e-6 {
            return 41;
        }
        let set = bd::MassData { mass: 5.0, center: [0.25, 0.0], rotational_inertia: 7.0 };
        bd::set_mass_data(b, set);
        let got = bd::mass_data(b);
        if (got.mass - 5.0).abs() > 1e-5 || (got.center[0] - 0.25).abs() > 1e-5 || (got.rotational_inertia - 7.0).abs() > 1e-5 {
            return 42;
        }
        b2Glue_DestroyWorld(w);

        // User data: 0 by default, then whatever was stored, including the full u32 range.
        let w = b2Glue_CreateWorld(0.0, -10.0, 0, 0);
        let b = dynamic_body(w, 0.0, 0.0);
        if bd::user_data(b) != 0 {
            return 50;
        }
        bd::set_user_data(b, 4242);
        if bd::user_data(b) != 4242 {
            return 51;
        }
        bd::set_user_data(b, u32::MAX);
        if bd::user_data(b) != u32::MAX {
            return 52;
        }
        b2Glue_DestroyWorld(w);

        // Enable and disable: a disabled body does not move, and falls again once enabled.
        let w = b2Glue_CreateWorld(0.0, -10.0, 0, 0);
        let b = dynamic_body(w, 0.0, 10.0);
        b2Glue_AddCircle(b, 0.5, 1.0, 0.5, 0.0, 0);
        let before = position(b);
        bd::disable(b);
        if bd::is_enabled(b) {
            return 60;
        }
        for _ in 0..30 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
        }
        if position(b) != before {
            return 61;
        }
        bd::enable(b);
        if !bd::is_enabled(b) {
            return 62;
        }
        for _ in 0..30 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
        }
        if before[1] - position(b)[1] < 1.0 {
            return 63; // 0.5 s of free fall is 1.25 m
        }
        b2Glue_DestroyWorld(w);

        // Impulses by batch: v = J / m for linear, w = J / I for angular, each body gets its own entry.
        let w = b2Glue_CreateWorld(0.0, 0.0, 0, 0);
        let ids = [dynamic_body(w, 0.0, 0.0), dynamic_body(w, 5.0, 0.0), dynamic_body(w, 10.0, 0.0)];
        for &id in &ids {
            b2Glue_AddCircle(id, 0.5, 1.0, 0.5, 0.0, 0);
        }
        let linear = [[1.0f32, 0.0], [0.0, 2.0], [-3.0, 4.0]];
        bd::apply_linear_impulses(&ids, &linear, true);
        let mut state = [0.0f32; 18];
        b2Glue_GetBatchState(ids.as_ptr(), 3, state.as_mut_ptr());
        for i in 0..3 {
            let mass = bd::mass_data(ids[i]).mass;
            for k in 0..2 {
                let expected = linear[i][k] / mass;
                if (state[i * 6 + 3 + k] - expected).abs() > 1e-4 * expected.abs().max(1.0) {
                    return 70;
                }
            }
        }
        let angular = [0.5f32, -0.5, 0.0];
        bd::apply_angular_impulses(&ids, &angular, true);
        b2Glue_GetBatchState(ids.as_ptr(), 3, state.as_mut_ptr());
        for i in 0..3 {
            let inertia = bd::mass_data(ids[i]).rotational_inertia;
            let expected = angular[i] / inertia;
            if (state[i * 6 + 5] - expected).abs() > 1e-4 * expected.abs().max(1.0) {
                return 71;
            }
        }
        b2Glue_DestroyWorld(w);
    }
    0
}

fn filters() -> i32 {
    unsafe {
        let w = b2Glue_CreateWorld(0.0, -10.0, 0, 0);

        // Floors at x = 0 and x = 10 and a floor at x = 20 with a negative group, each 6 m wide, top at y = 0.5.
        let floor_a = static_body(w, 0.0, 0.0);
        let shape_a = b2Glue_AddBox(floor_a, 3.0, 0.5, 1.0, 0.5, 0.0, 0);
        let floor_b = static_body(w, 10.0, 0.0);
        b2Glue_AddBox(floor_b, 3.0, 0.5, 1.0, 0.5, 0.0, 0);
        let floor_c = static_body(w, 20.0, 0.0);
        let shape_c = b2Glue_AddBox(floor_c, 3.0, 0.5, 1.0, 0.5, 0.0, 0);

        if fl::filter(shape_a) != fl::Filter::default() {
            return 80;
        }
        // Floor A collides only with category 2. Floor C has group -1, and so does its ball.
        fl::set_filter(shape_a, fl::Filter { category_bits: 2, mask_bits: 2, group_index: 0 });
        fl::set_filter(shape_c, fl::Filter { category_bits: 1, mask_bits: u64::MAX, group_index: -1 });
        let fa = fl::filter(shape_a);
        if fa.category_bits != 2 || fa.mask_bits != 2 || fa.group_index != 0 {
            return 81;
        }
        if fl::filter(shape_c).group_index != -1 {
            return 82;
        }

        // Balls start 3 m above each floor. Ball A (category 1) cannot touch floor A, ball B hits floor B,
        // ball C shares group -1 with floor C and passes through it.
        let ball_a = dynamic_body(w, 0.0, 3.0);
        let ball_b = dynamic_body(w, 10.0, 3.0);
        let ball_c = dynamic_body(w, 20.0, 3.0);
        let shape_ball_a = b2Glue_AddCircle(ball_a, 0.5, 1.0, 0.5, 0.0, 0);
        b2Glue_AddCircle(ball_b, 0.5, 1.0, 0.5, 0.0, 0);
        let shape_ball_c = b2Glue_AddCircle(ball_c, 0.5, 1.0, 0.5, 0.0, 0);
        fl::set_filter(shape_ball_c, fl::Filter { category_bits: 1, mask_bits: u64::MAX, group_index: -1 });
        if fl::filter(shape_ball_a) != fl::Filter::default() {
            return 83;
        }

        let ids = [ball_a, ball_b, ball_c];
        for _ in 0..120 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
        }
        let mut out = [0.0f32; 9];
        b2Glue_GetBatchTransforms(ids.as_ptr(), 3, out.as_mut_ptr());
        let (y_a, y_b, y_c) = (out[1], out[4], out[7]);
        if y_a > -5.0 {
            return 84; // ball A should have fallen through floor A
        }
        if y_c > -5.0 {
            return 85; // ball C should have fallen through floor C (same negative group)
        }
        if !(y_b > 0.9 && y_b < 1.2) {
            return 86; // ball B should rest on floor B, center at about 1.0
        }
        b2Glue_DestroyWorld(w);
    }
    0
}

#[no_mangle]
pub extern "C" fn run_queries() -> i32 {
    queries()
}

#[no_mangle]
pub extern "C" fn run_bodies() -> i32 {
    bodies()
}

#[no_mangle]
pub extern "C" fn run_filters() -> i32 {
    filters()
}

#[no_mangle]
pub extern "C" fn run() -> i32 {
    for section in [shapes, joints, queries, bodies, filters] {
        let code = section();
        if code != 0 {
            return code;
        }
    }
    let (w, ids) = build();
    unsafe {
        let mut events = 0;
        let mut buf = vec![0u64; 2 * 256];
        for _ in 0..120 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
            events += b2Glue_ContactBeginEvents(w, buf.as_mut_ptr(), 256);
        }
        if events == 0 {
            return 1; // no contact events: objects never hit the floor
        }

        // Take snapshot
        let snap_size = b2Glue_SnapshotSize(w);
        if snap_size <= 0 {
            return 2;
        }
        let mut snapshot_bytes = vec![0u8; snap_size as usize];
        let written = b2Glue_SnapshotTake(w, snapshot_bytes.as_mut_ptr(), snap_size);
        if written <= 0 {
            return 3;
        }

        // Advance 60 steps
        for _ in 0..60 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
        }
        let state_after_step = state(&ids);

        // Restore snapshot into world in-place
        if !b2Glue_SnapshotRestore(w, snapshot_bytes.as_ptr(), written) {
            return 4;
        }

        // Advance 60 steps again from restored point
        for _ in 0..60 {
            b2Glue_Step(w, 1.0 / 60.0, 4);
        }
        let state_after_restore = state(&ids);

        if state_after_step != state_after_restore {
            return 5; // restored world diverged
        }

        // Test raycast
        let origin = [0.0f32, 50.0];
        let translation = [0.0f32, -100.0];
        let mut hit = [0.0f32; 6];
        let hit_shape = b2Glue_CastRayClosest(w, origin.as_ptr(), translation.as_ptr(), hit.as_mut_ptr());
        if hit_shape == 0 || hit[0] >= 1.0 {
            return 6; // ray missed the pile
        }

        // Test character mover
        let mut char_pos = [0.0f32, 25.0];
        let delta = [1.0f32, -5.0];
        let grounded = b2Glue_MoveCharacter(w, 0, char_pos.as_mut_ptr(), 0.5, 0.3, delta.as_ptr(), 0.7);
        if char_pos[1] >= 25.0 {
            return 7; // character did not move downwards
        }
        let _ = grounded;

        b2Glue_DestroyWorld(w);
    }
    0
}

#[test]
fn test_smoke() {
    assert_eq!(run(), 0);
}
