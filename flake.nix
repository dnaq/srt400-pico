{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { self, nixpkgs, utils, rust-overlay }:
    utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; overlays = [ rust-overlay.overlay ];};
        rust = pkgs.rust-bin.stable.latest.default.override {
          targets = [ "thumbv6m-none-eabi" ];
        };
      in
      {
        devShell = with pkgs; mkShell {
          buildInputs = [ rust elf2uf2-rs tio picotool ];
        };
      });
}
