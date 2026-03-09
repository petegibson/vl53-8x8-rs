use std::env;
use std::path::PathBuf;

fn main() {
    // Tell rustc that we are creating a custom `cfg` flag. This is required to
    // prevent a warning about an unexpected `cfg` condition name.
    println!("cargo:rustc-check-cfg=cfg(motion_or_thresholds)");

    // Directory containing the ST ULD source files
    let driver_dir = PathBuf::from("src").join("VL53L8CX_ULD_API");

    // Start with the base API file, which is always required.
    let mut c_sources = vec![driver_dir.join("src").join("vl53l8cx_api.c")];
    let mut cc_build = cc::Build::new();

    // Determine the number of targets based on active features.
    let mut nb_targets = "1"; // Default
    if env::var("CARGO_FEATURE_TARGETS_4").is_ok() {
        nb_targets = "4";
    } else if env::var("CARGO_FEATURE_TARGETS_3").is_ok() {
        nb_targets = "3";
    } else if env::var("CARGO_FEATURE_TARGETS_2").is_ok() {
        nb_targets = "2";
    }
    // Note: If multiple `targets-*` features are enabled, the highest number will be used.
    // For true mutual exclusion, a more complex check could be added here to panic the build.

    let mut bindgen_builder = bindgen::Builder::default();

    // --- Feature: xtalk ---
    if env::var("CARGO_FEATURE_XTALK").is_ok() {
        println!("cargo:rustc-cfg=feature=\"xtalk\"");
        c_sources.push(driver_dir.join("src").join("vl53l8cx_plugin_xtalk.c"));
        cc_build.define("VL53L8CX_USE_XTALK", "1");
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_USE_XTALK=1");
    }

    // --- Feature: motion & thresholds ---
    // The motion indicator plugin contains code for both motion and detection thresholds.
    if env::var("CARGO_FEATURE_MOTION").is_ok() || env::var("CARGO_FEATURE_THRESHOLDS").is_ok() {
        println!("cargo:rustc-cfg=motion_or_thresholds");
        c_sources.push(driver_dir.join("src").join("vl53l8cx_plugin_motion_indicator.c"));
        cc_build.define("VL53L8CX_USE_MOTION_INDICATOR", "1");
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_USE_MOTION_INDICATOR=1");
    }

    // If the motion feature is disabled, we must tell BOTH the C compiler and
    // bindgen to remove the `motion_indicator` field from VL53L8CX_ResultsData.
    // This also affects VL53L8CX_MAX_RESULTS_SIZE and thus temp_buffer size —
    // a mismatch causes buffer overflows at runtime.
    if env::var("CARGO_FEATURE_MOTION").is_err() {
        cc_build.define("VL53L8CX_DISABLE_MOTION_INDICATOR", "1");
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_MOTION_INDICATOR");
    }

// --- Feature: Raw Firmware Format ---
if env::var("CARGO_FEATURE_RAW_FORMAT").is_ok() {
    cc_build.define("VL53L8CX_USE_RAW_FORMAT", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_USE_RAW_FORMAT=1");
}

// --- Output Disablement Features for Size Optimization (maps to VL53L8CX_DISABLE_... macros) ---
if env::var("CARGO_FEATURE_DISABLE_AMBIENT").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_AMBIENT_PER_SPAD", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_AMBIENT_PER_SPAD");
}
if env::var("CARGO_FEATURE_DISABLE_SPADS").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_NB_SPADS_ENABLED", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_NB_SPADS_ENABLED");
}
if env::var("CARGO_FEATURE_DISABLE_NB_TARGETS").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_NB_TARGET_DETECTED", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_NB_TARGET_DETECTED");
}
if env::var("CARGO_FEATURE_DISABLE_SIGNAL").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_SIGNAL_PER_SPAD", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_SIGNAL_PER_SPAD");
}
if env::var("CARGO_FEATURE_DISABLE_RANGE_SIGMA").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_RANGE_SIGMA_MM", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_RANGE_SIGMA_MM");
}
if env::var("CARGO_FEATURE_DISABLE_DISTANCE").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_DISTANCE_MM", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_DISTANCE_MM");
}
if env::var("CARGO_FEATURE_DISABLE_TARGET_STATUS").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_TARGET_STATUS", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_TARGET_STATUS");
}
if env::var("CARGO_FEATURE_DISABLE_REFLECTANCE").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_REFLECTANCE_PERCENT", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_REFLECTANCE_PERCENT");
}
if env::var("CARGO_FEATURE_DISABLE_MOTION_INDICATOR_OUTPUT").is_ok() {
    cc_build.define("VL53L8CX_DISABLE_MOTION_INDICATOR", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L8CX_DISABLE_MOTION_INDICATOR");
}

    // --- Configuration for Cross-Compilation ---
    let target = env::var("TARGET").unwrap();
    if target.starts_with("armv7-unknown-linux-") {
        // When cross-compiling, both `cc` and `bindgen` need to know where the
        // target's C standard library headers are. We can ask the C compiler for its sysroot.
        let compiler = cc_build.get_compiler();
        let output = std::process::Command::new(compiler.path())
            .arg("-print-sysroot")
            .output()
            .expect("Failed to execute C compiler to get sysroot");

        if !output.status.success() {
            panic!(
                "C compiler failed to report sysroot. stderr: {}",
                String::from_utf8_lossy(&output.stderr)
            );
        }

        let sysroot = String::from_utf8(output.stdout).unwrap().trim().to_string();

        bindgen_builder = bindgen_builder
            .clang_arg(format!("--sysroot={}", sysroot))
            .clang_arg(format!("--target={}", target));
    } else if target.starts_with("thumbv") {
        // Bare-metal ARM cross-compilation (e.g. thumbv7em-none-eabihf).
        // The CC_<target> and AR_<target> env vars tell the `cc` crate which
        // cross-compiler to use. We query it for include paths so that
        // bindgen's clang can find stdint.h, string.h, etc.
        let compiler = cc_build.get_compiler();

        bindgen_builder = bindgen_builder
            .clang_arg(format!("--target={}", target))
            .clang_arg("-ffreestanding");

        // GCC built-in headers (stdint.h, stddef.h, etc.)
        if let Ok(output) = std::process::Command::new(compiler.path())
            .arg("-print-file-name=include")
            .output()
        {
            let p = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if !p.is_empty() && p != "include" {
                bindgen_builder = bindgen_builder.clang_arg(format!("-isystem{}", p));
            }
        }

        // C library headers from the cross-toolchain (newlib string.h, etc.)
        if let Ok(output) = std::process::Command::new(compiler.path())
            .arg("-print-sysroot")
            .output()
        {
            let p = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if !p.is_empty() {
                bindgen_builder = bindgen_builder.clang_arg(format!("-isystem{}/include", p));
            }
        }
    }

    // Compile the C sources into a static library
    cc_build
        .files(c_sources)
        .include(driver_dir.join("inc"))
        // Suppress signed/unsigned comparison warning in ST's SwapBuffer macro
        .flag("-Wno-sign-compare")
        // Configure driver settings.
        .define("VL53L8CX_NB_TARGET_PER_ZONE", nb_targets)
        // This tells the C driver to use the high-level platform functions.
        .define("USE_ULP_PLATFORM", "1")
        .compile("vl53l8cx_driver");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/VL53L8CX_ULD_API/inc/vl53l8cx_api.h");

    // Generate the Rust bindings using bindgen
    let bindings = bindgen_builder
        .header(driver_dir.join("inc").join("vl53l8cx_api.h").to_str().unwrap())
        // Explicitly include plugin headers so bindgen can find all types.
        .header(driver_dir.join("inc").join("vl53l8cx_plugin_xtalk.h").to_str().unwrap())
        .header(driver_dir.join("inc").join("vl53l8cx_plugin_motion_indicator.h").to_str().unwrap())
        .header(driver_dir.join("inc").join("vl53l8cx_plugin_detection_thresholds.h").to_str().unwrap())
        // Add the include path for bindgen's clang parser.
        .clang_arg(format!("-I{}", driver_dir.join("inc").display()))
        // Also define the macros for bindgen's C parser.
        .clang_arg(format!("-DVL53L8CX_NB_TARGET_PER_ZONE={}", nb_targets))
        .clang_arg("-DUSE_ULP_PLATFORM=1")
        // Generate no_std compatible bindings
        .use_core()
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Whitelist the functions, types, and constants we want to use.
        .allowlist_function("vl53l8cx_.*")
        .allowlist_type("VL53L8CX_.*")
        .allowlist_var("VL53L8CX_.*")
        .allowlist_var("VL53L8CX_XTALK_BUFFER_SIZE") // Ensure this const is always generated
        .generate()
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
