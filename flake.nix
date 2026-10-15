{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs { inherit system overlays; };
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          targets = [
            "thumbv7em-none-eabihf"
            "aarch64-unknown-linux-gnu"      # 64-bit Raspberry Pi OS Bookworm — needs nix-ld on NixOS
            "aarch64-unknown-linux-musl"     # 64-bit, statically linked — runs anywhere
            "armv7-unknown-linux-gnueabihf"  # 32-bit Raspberry Pi OS (legacy)
          ];
          extensions = [ "llvm-tools" ];
        };
        # Cross-toolchains for the Pi targets.
        crossAarch64 = pkgs.pkgsCross.aarch64-multiplatform;
        crossAarch64Musl = pkgs.pkgsCross.aarch64-multiplatform-musl;
        crossArmv7 = pkgs.pkgsCross.armv7l-hf-multiplatform;
      in {
        devShells.default = pkgs.mkShell {
          nativeBuildInputs = [
            pkgs.pkg-config
          ];
          buildInputs = [
            rustToolchain
            pkgs.gcc-arm-embedded
            pkgs.llvmPackages.libclang
            pkgs.cargo-binutils
            pkgs.probe-rs-tools
            pkgs.just
            # Pi cross-compilers (provide aarch64-unknown-linux-gnu-gcc /
            # armv7l-unknown-linux-gnueabihf-gcc plus their binutils & sysroots).
            crossAarch64.stdenv.cc
            crossAarch64Musl.stdenv.cc
            crossArmv7.stdenv.cc
          ];

          env = {
            CC_thumbv7em_none_eabihf = "arm-none-eabi-gcc";
            AR_thumbv7em_none_eabihf = "arm-none-eabi-ar";
            LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";

            # 64-bit Raspberry Pi OS target (glibc, dynamically linked).
            CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER = "${crossAarch64.stdenv.cc}/bin/${crossAarch64.stdenv.cc.targetPrefix}gcc";
            CC_aarch64_unknown_linux_gnu = "${crossAarch64.stdenv.cc}/bin/${crossAarch64.stdenv.cc.targetPrefix}gcc";
            AR_aarch64_unknown_linux_gnu = "${crossAarch64.stdenv.cc}/bin/${crossAarch64.stdenv.cc.targetPrefix}ar";

            # 64-bit aarch64 musl (statically linked, runs on any Linux including NixOS).
            CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER = "${crossAarch64Musl.stdenv.cc}/bin/${crossAarch64Musl.stdenv.cc.targetPrefix}gcc";
            CC_aarch64_unknown_linux_musl = "${crossAarch64Musl.stdenv.cc}/bin/${crossAarch64Musl.stdenv.cc.targetPrefix}gcc";
            AR_aarch64_unknown_linux_musl = "${crossAarch64Musl.stdenv.cc}/bin/${crossAarch64Musl.stdenv.cc.targetPrefix}ar";

            # 32-bit Raspberry Pi OS target.
            CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER = "${crossArmv7.stdenv.cc}/bin/${crossArmv7.stdenv.cc.targetPrefix}gcc";
            CC_armv7_unknown_linux_gnueabihf = "${crossArmv7.stdenv.cc}/bin/${crossArmv7.stdenv.cc.targetPrefix}gcc";
            AR_armv7_unknown_linux_gnueabihf = "${crossArmv7.stdenv.cc}/bin/${crossArmv7.stdenv.cc.targetPrefix}ar";
          };

          shellHook = ''
            export SHELL=${pkgs.bashInteractive}/bin/bash
          '';
        };
      }
    );
}
