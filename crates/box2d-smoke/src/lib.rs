//! Runs Box2D inside a plain wasm32-unknown-unknown module: drop boxes/balls on a floor, step,
//! test contacts, snapshot, restore, and raycast. `run` returns 0 when everything holds,
//! or a code naming the failed check.
use box2d_sys::*;

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

#[no_mangle]
pub extern "C" fn run() -> i32 {
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
