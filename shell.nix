
let 
  pkgs = import <nixpkgs> { };
in 
pkgs.mkShellNoCC {
  packages = with pkgs; [
    
        
  ];

    LD_LIBRARY_PATH = "${pkgs.stdenv.cc.cc.lib}/lib:${pkgs.xorg.libX11}/lib";

    shellHook = "ln -s ${pkgs.stdenv.cc.cc.lib}/lib/libstdc++.so.6 build/jni/release/
    ln -s ${pkgs.xorg.libX11}/lib/libX11.so.6 build/jni/release/";
    JAVA_HOME = "/home/xlfrie/Downloads/WPILib_Linux-2026.2.1/jdk/";
}