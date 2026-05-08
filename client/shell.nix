{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  packages = [
    (pkgs.python3.withPackages (ps: with ps; [
      pyqtgraph
      pyside6
      pyopengl
      numpy
      requests
    ]))
  ];
}
