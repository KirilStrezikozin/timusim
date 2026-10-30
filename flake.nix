{
  description = "Flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";

    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };

        mkFormatterInputs =
          pkgs: with pkgs; [
            deadnix
            statix
            alejandra
            llvmPackages_19.clang-tools
            shellcheck
            fd
            nixfmt-rfc-style
            ruff
          ];

        mkLspsInputs =
          pkgs: with pkgs; [
            cmake-language-server
            nixd
            pyright
          ];
      in
      {
        devShells = rec {
          # Firmware (PlatformIO), host simulator (CMake) and Python analysis (uv).
          # Usage: `nix develop .#platformio` or `nix develop`.
          platformio = pkgs.mkShell {
            nativeBuildInputs =
              with pkgs;
              [
                gcc-arm-embedded-13
                tio # Terminal program to interface with serial.
                openocd
                stlink
                picocom
                pkgs.platformio

                cmake
                gnumake

                python312
                uv

                tree
                pre-commit
              ]
              ++ (mkFormatterInputs pkgs)
              ++ (mkLspsInputs pkgs);

            # Wheels from PyPI (numpy, scipy) link against these at runtime.
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
              pkgs.stdenv.cc.cc.lib
              pkgs.zlib
            ];

            UV_PYTHON = "${pkgs.python312}/bin/python3";
            UV_PYTHON_DOWNLOADS = "never";

            shellHook = ''
              # Packages from a Python elsewhere on PYTHONPATH shadow the venv's.
              unset PYTHONPATH
              if [ -f .venv/bin/activate ]; then
                source .venv/bin/activate
              fi
            '';
          };

          matlab = pkgs.mkShell {
            nativeBuildInputs =
              with pkgs;
              [
                python312
                uv

                tree
                pre-commit

                distrobox
              ]
              ++ (mkFormatterInputs pkgs)
              ++ (mkLspsInputs pkgs);

            # Wheels from PyPI (numpy, scipy) link against these at runtime.
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
              pkgs.stdenv.cc.cc.lib
              pkgs.zlib
            ];

            UV_PYTHON = "${pkgs.python312}/bin/python3";
            UV_PYTHON_DOWNLOADS = "never";

            shellHook = ''
              # Packages from a Python elsewhere on PYTHONPATH shadow the venv's.
              unset PYTHONPATH
              if [ -f .venv/bin/activate ]; then
                source .venv/bin/activate
              fi
            '';
          };

          default = platformio;
        };

        # For compatibility with older versions of the `nix` binary
        devShell = self.devShells.${system}.default;

        # Formatter to use with the `nix fmt` command.
        formatter = pkgs.nixfmt-tree;
      }
    );
}
