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
          targets = [ "thumbv7em-none-eabihf" ];
          extensions = [ "llvm-tools" ];
        };
      in {
        devShells.default = pkgs.mkShell {
          buildInputs = [
            rustToolchain
            pkgs.gcc-arm-embedded
            pkgs.llvmPackages.libclang
            pkgs.cargo-binutils
            pkgs.probe-rs-tools
          ];

          env = {
            CC_thumbv7em_none_eabihf = "arm-none-eabi-gcc";
            AR_thumbv7em_none_eabihf = "arm-none-eabi-ar";
            LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
          };

          shellHook = ''
            export SHELL=${pkgs.bashInteractive}/bin/bash
          '';
        };
      }
    );
}
