// Compiles Box2D and the glue with clang. On wasm32-unknown-unknown there is no libc, so a tiny one comes from
// include/libc + src/shim.c + src/lib.rs, and the SSE2 intrinsics Box2D uses map to WASM SIMD through include/sse.
// Needs clang with the wasm32 target on PATH (CC_wasm32_unknown_unknown=clang).
use std::path::PathBuf;

fn main() {
    let here = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let repo = here.join("../..");
    let box2d = repo.join("vendor/box2d");
    assert!(box2d.join("src/body.c").exists(), "vendor/box2d is empty: run `git submodule update --init`");

    let target = std::env::var("TARGET").unwrap();
    let wasm = target == "wasm32-unknown-unknown";

    let mut b = cc::Build::new();
    b.opt_level(3).warnings(false).include(box2d.join("include")).include(box2d.join("src")).include(repo.join("src/c"));
    if b.get_compiler().is_like_msvc() {
        b.flag("/std:c17").flag("/experimental:c11atomics");
    } else {
        b.std("gnu17");
        b.flag("-ffp-contract=off");
    }
    if wasm {
        b.flag("-ffreestanding").flag("-msimd128").define("__SSE__", "1").define("__SSE2__", "1");
        b.define("B2_CPU_WASM", None);
        // libc headers last-resort first: our shims must win over any host headers clang might find.
        b.flag("-nostdlibinc").include(here.join("include/libc")).include(here.join("include/sse"));
        // A module linked with --shared-memory (threads) wants every object built with atomics and bulk memory.
        let flags = std::env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
        if flags.contains("+atomics") {
            b.flag("-matomics").flag("-mbulk-memory");
        }
    }
    if std::env::var("BOX2D_SCALAR").is_ok() {
        b.define("BOX2D_DISABLE_SIMD", None);
    }

    let mut timer = b.clone();
    let mut platform = b.clone();
    // Upstream picks the 8-wide solver by file: contact_solver_w8.c needs AVX2. clang and gcc enable it for that file with a target
    // attribute, but MSVC needs the flag, so that one file builds on its own with /arch:AVX2 (every other file stays SSE2).
    let mut avx2 = b.clone();
    let msvc = b.get_compiler().is_like_msvc();
    let mut has_avx2_file = false;
    for entry in std::fs::read_dir(box2d.join("src")).unwrap() {
        let p = entry.unwrap().path();
        if p.extension().map_or(false, |e| e == "c") {
            if wasm && p.file_name().map_or(false, |n| n == "timer.c") {
                continue;
            }
            if msvc && !wasm && p.file_name().map_or(false, |n| n == "contact_solver_w8.c") {
                avx2.flag("/arch:AVX2").file(p);
                has_avx2_file = true;
                continue;
            }
            b.file(p);
        }
    }
    if has_avx2_file {
        avx2.compile("box2d_avx2");
    }
    if wasm {
        for name in ["CreateMutex", "DestroyMutex", "LockMutex", "UnlockMutex", "CreateSemaphore", "DestroySemaphore", "WaitSemaphore", "SignalSemaphore"] {
            let renamed = format!("b2{name}Unused");
            timer.define(&format!("b2{name}"), Some(renamed.as_str()));
        }
        timer.file(box2d.join("src/timer.c"));
        timer.compile("box2d_timer");
        platform.file(here.join("src/platform.c"));
        platform.compile("box2d_platform");
        b.file(here.join("src/shim.c"));
    }
    b.file(repo.join("src/c/box2d_glue.c"));
    for area in ["bodies", "filters", "joints", "queries", "shapes"] {
        b.file(repo.join(format!("src/c/box2d_glue_{area}.c")));
    }
    b.compile("box2d");

    println!("cargo:rerun-if-env-changed=CARGO_ENCODED_RUSTFLAGS");
    println!("cargo:rerun-if-changed=../../src/c");
    println!("cargo:rerun-if-changed=src/shim.c");
    println!("cargo:rerun-if-changed=../../vendor/box2d/src");
}
