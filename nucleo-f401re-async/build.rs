fn main() {
    // Linker scripts for binary targets
    println!("cargo::rustc-link-arg-bins=-Tlink.x");
    println!("cargo::rustc-link-arg-bins=-Tdefmt.x");

    // Linker scripts for test targets
    println!("cargo::rustc-link-arg-tests=-Tlink.x");
    println!("cargo::rustc-link-arg-tests=-Tdefmt.x");
    println!("cargo::rustc-link-arg-tests=-Tembedded-test.x");
}
