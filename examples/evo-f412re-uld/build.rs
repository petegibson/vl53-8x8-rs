use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put memory.x on the linker search path (stm32f4xx-hal ships none).
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    // embedded-test 0.7.2's `#[tests]` macro emits `cfg(rust_analyzer)`;
    // declare it so rustc does not warn about an unexpected cfg name.
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");

    // Linker scripts for the demo binary
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rustc-link-arg-bins=-Tdefmt.x");

    // Linker scripts for embedded-test on-target test binaries
    println!("cargo::rustc-link-arg-tests=-Tlink.x");
    println!("cargo::rustc-link-arg-tests=-Tdefmt.x");
    println!("cargo::rustc-link-arg-tests=-Tembedded-test.x");
}
