fn main() {
    // Linker scripts for binary targets. The memory layout is supplied by
    // embassy-stm32's `memory-x` feature for the selected chip (STM32F412RE).
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rustc-link-arg-bins=-Tdefmt.x");

    // Linker scripts for embedded-test on-target test binaries.
    println!("cargo::rustc-link-arg-tests=-Tlink.x");
    println!("cargo::rustc-link-arg-tests=-Tdefmt.x");
    println!("cargo::rustc-link-arg-tests=-Tembedded-test.x");
}
