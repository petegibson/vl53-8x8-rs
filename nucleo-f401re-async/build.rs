fn main() {
    // embedded-test 0.7.2's `#[tests]` macro emits `cfg(rust_analyzer)`;
    // declare it so rustc does not warn about an unexpected cfg name.
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");

    // Linker scripts for binary targets
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rustc-link-arg-bins=-Tdefmt.x");

    // Linker scripts for test targets
    println!("cargo::rustc-link-arg-tests=-Tlink.x");
    println!("cargo::rustc-link-arg-tests=-Tdefmt.x");
    println!("cargo::rustc-link-arg-tests=-Tembedded-test.x");
}
