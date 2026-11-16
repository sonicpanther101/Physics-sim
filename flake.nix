{
  description = "SDL3 dot";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [ gcc pkg-config sdl3 glm ];
        CPATH = pkgs.lib.makeSearchPathOutput "dev" "include" [ pkgs.sdl3 pkgs.glm ];
      };
    };
}
