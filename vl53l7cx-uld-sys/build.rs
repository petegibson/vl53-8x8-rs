use std::env;
use std::path::PathBuf;

fn main() {
    // Tell rustc that we are creating a custom `cfg` flag. This is required to
    // prevent a warning about an unexpected `cfg` condition name.
    println!("cargo:rustc-check-cfg=cfg(motion_or_thresholds)");

    // Directory containing the ST ULD source files
    let driver_dir = PathBuf::from("src").join("VL53L7CX_ULD_API");

    // Start with the base API file, which is always required.
    let mut c_sources = vec![driver_dir.join("src").join("vl53l7cx_api.c")];
    let mut cc_build = cc::Build::new();

    // Determine VL53L7CX_NB_TARGET_PER_ZONE from the `targets-N` features.
    // Exactly one may be enabled; `targets-1` is the same as the default.
    // Cargo unions features across the dependency graph, so two crates asking
    // for different values is a real conflict, not something to resolve by
    // picking the highest: the C struct layout would silently change for one
    // of them.
    let selected: Vec<(&str, u8)> = [
        ("targets-1", 1u8),
        ("targets-2", 2),
        ("targets-3", 3),
        ("targets-4", 4),
    ]
    .into_iter()
    .filter(|(feature, _)| {
        env::var(format!(
            "CARGO_FEATURE_{}",
            feature.to_uppercase().replace('-', "_")
        ))
        .is_ok()
    })
    .collect();
    let nb_targets: u8 = match selected.as_slice() {
        [] => 1,
        [(_, n)] => *n,
        many => {
            let names: Vec<&str> = many.iter().map(|(f, _)| *f).collect();
            panic!(
                "vl53l7cx-uld-sys: conflicting `targets-N` features enabled: {}. \
                 VL53L7CX_NB_TARGET_PER_ZONE can only have one value; enable exactly one of \
                 `targets-1`, `targets-2`, `targets-3`, `targets-4` (or none for the default of 1) \
                 across the whole dependency graph.",
                names.join(", ")
            );
        }
    };
    let nb_targets = nb_targets.to_string();
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    // Bindgen does not emit macros passed on the command line, so publish the
    // selected value as a Rust constant next to the generated bindings.
    std::fs::write(
        out_path.join("nb_targets.rs"),
        format!(
            "/// Number of targets reported per zone (`VL53L7CX_NB_TARGET_PER_ZONE`),\n\
             /// selected by the `targets-N` cargo features (default 1).\n\
             ///\n\
             /// Per-target arrays in [`VL53L7CX_ResultsData`] (`distance_mm`, `target_status`,\n\
             /// `signal_per_spad`, `range_sigma_mm`, `reflectance`) hold `64 * N` entries,\n\
             /// indexed `zone * VL53L7CX_NB_TARGET_PER_ZONE + target`.\n\
             pub const VL53L7CX_NB_TARGET_PER_ZONE: usize = {};\n",
            nb_targets
        ),
    )
    .expect("Couldn't write nb_targets.rs");

    let mut bindgen_builder = bindgen::Builder::default();

    // --- Feature: xtalk ---
    if env::var("CARGO_FEATURE_XTALK").is_ok() {
        println!("cargo:rustc-cfg=feature=\"xtalk\"");
        c_sources.push(driver_dir.join("src").join("vl53l7cx_plugin_xtalk.c"));
        cc_build.define("VL53L7CX_USE_XTALK", "1");
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_USE_XTALK=1");
    }

    // --- Feature: motion & thresholds ---
    // The motion indicator plugin contains code for both motion and detection thresholds.
    if env::var("CARGO_FEATURE_MOTION").is_ok() || env::var("CARGO_FEATURE_THRESHOLDS").is_ok() {
        println!("cargo:rustc-cfg=motion_or_thresholds");
        c_sources.push(driver_dir.join("src").join("vl53l7cx_plugin_motion_indicator.c"));
        c_sources.push(driver_dir.join("src").join("vl53l7cx_plugin_detection_thresholds.c"));
        cc_build.define("VL53L7CX_USE_MOTION_INDICATOR", "1");
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_USE_MOTION_INDICATOR=1");
    }

    // If the motion feature is disabled, we must tell BOTH the C compiler and
    // bindgen to remove the `motion_indicator` field from VL53L7CX_ResultsData.
    // This also affects VL53L7CX_MAX_RESULTS_SIZE and thus temp_buffer size —
    // a mismatch causes buffer overflows at runtime.
    if env::var("CARGO_FEATURE_MOTION").is_err() {
        cc_build.define("VL53L7CX_DISABLE_MOTION_INDICATOR", "1");
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_MOTION_INDICATOR");
    }

// --- Feature: Raw Firmware Format ---
if env::var("CARGO_FEATURE_RAW_FORMAT").is_ok() {
    cc_build.define("VL53L7CX_USE_RAW_FORMAT", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_USE_RAW_FORMAT=1");
}

// --- Output Disablement Features for Size Optimization (maps to VL53L7CX_DISABLE_... macros) ---
if env::var("CARGO_FEATURE_DISABLE_AMBIENT").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_AMBIENT_PER_SPAD", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_AMBIENT_PER_SPAD");
}
if env::var("CARGO_FEATURE_DISABLE_SPADS").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_NB_SPADS_ENABLED", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_NB_SPADS_ENABLED");
}
if env::var("CARGO_FEATURE_DISABLE_NB_TARGETS").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_NB_TARGET_DETECTED", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_NB_TARGET_DETECTED");
}
if env::var("CARGO_FEATURE_DISABLE_SIGNAL").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_SIGNAL_PER_SPAD", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_SIGNAL_PER_SPAD");
}
if env::var("CARGO_FEATURE_DISABLE_RANGE_SIGMA").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_RANGE_SIGMA_MM", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_RANGE_SIGMA_MM");
}
if env::var("CARGO_FEATURE_DISABLE_DISTANCE").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_DISTANCE_MM", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_DISTANCE_MM");
}
if env::var("CARGO_FEATURE_DISABLE_TARGET_STATUS").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_TARGET_STATUS", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_TARGET_STATUS");
}
if env::var("CARGO_FEATURE_DISABLE_REFLECTANCE").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_REFLECTANCE_PERCENT", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_REFLECTANCE_PERCENT");
}
if env::var("CARGO_FEATURE_DISABLE_MOTION_INDICATOR_OUTPUT").is_ok() {
    cc_build.define("VL53L7CX_DISABLE_MOTION_INDICATOR", "1");
    bindgen_builder = bindgen_builder.clang_arg("-DVL53L7CX_DISABLE_MOTION_INDICATOR");
}

    // --- Configuration for Cross-Compilation ---
    let target = env::var("TARGET").unwrap();
    let host = env::var("HOST").unwrap_or_default();
    let is_native_linux = target == host;
    let is_linux_cross = (target.starts_with("armv7-unknown-linux-")
        || target.starts_with("aarch64-unknown-linux-"))
        && target != host;

    if is_native_linux || is_linux_cross {
        // Native or Linux cross-compile. Bindgen uses libclang, which in
        // hermetic environments (nix, Docker) and most cross-toolchains
        // can't find libc headers on its own. Probe the (cross) C
        // compiler for the include paths it would actually search and
        // forward them to libclang via `-isystem`. For cross-compile we
        // also pass `--target` so clang parses headers for the right
        // architecture (sizes, alignments, predefined macros).
        let compiler = cc_build.get_compiler();
        if is_linux_cross {
            bindgen_builder = bindgen_builder.clang_arg(format!("--target={}", target));
        }
        if let Ok(output) = std::process::Command::new(compiler.path())
            .args(["-E", "-x", "c", "-v", "-"])
            .stdin(std::process::Stdio::null())
            .output()
        {
            let stderr = String::from_utf8_lossy(&output.stderr);
            let mut in_search = false;
            for line in stderr.lines() {
                if line.contains("#include <...> search starts here:") {
                    in_search = true;
                    continue;
                }
                if line.contains("End of search list.") {
                    break;
                }
                if in_search {
                    let path = line.trim();
                    if !path.is_empty() {
                        bindgen_builder = bindgen_builder.clang_arg(format!("-isystem{}", path));
                    }
                }
            }
        }
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
        .define("VL53L7CX_NB_TARGET_PER_ZONE", nb_targets.as_str())
        // This tells the C driver to use the high-level platform functions.
        .define("USE_ULP_PLATFORM", "1")
        .compile("vl53l7cx_uld");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/VL53L7CX_ULD_API/inc/vl53l7cx_api.h");

    // Generate the Rust bindings using bindgen
    let bindings = bindgen_builder
        .header(driver_dir.join("inc").join("vl53l7cx_api.h").to_str().unwrap())
        // Explicitly include plugin headers so bindgen can find all types.
        .header(driver_dir.join("inc").join("vl53l7cx_plugin_xtalk.h").to_str().unwrap())
        .header(driver_dir.join("inc").join("vl53l7cx_plugin_motion_indicator.h").to_str().unwrap())
        .header(driver_dir.join("inc").join("vl53l7cx_plugin_detection_thresholds.h").to_str().unwrap())
        // Add the include path for bindgen's clang parser.
        .clang_arg(format!("-I{}", driver_dir.join("inc").display()))
        // Also define the macros for bindgen's C parser.
        .clang_arg(format!("-DVL53L7CX_NB_TARGET_PER_ZONE={}", nb_targets))
        .clang_arg("-DUSE_ULP_PLATFORM=1")
        // Generate no_std compatible bindings
        .use_core()
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Whitelist the functions, types, and constants we want to use.
        .allowlist_function("vl53l7cx_.*")
        .allowlist_type("VL53L7CX_.*")
        .allowlist_var("VL53L7CX_.*")
        .allowlist_var("VL53L7CX_XTALK_BUFFER_SIZE") // Ensure this const is always generated
        .blocklist_item("VL53L7CX_GET_XTALK_CMD") // static const in C header; defined as Rust const in lib.rs
        .blocklist_item("VL53L7CX_CALIBRATE_XTALK") // static const in C header; not needed from Rust
        .generate()
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
