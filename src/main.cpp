#include <iostream>

#include <surface_subdivision/config.h>
#include <surface_subdivision/MeshViewer.h>

int main() {
    zone::MeshViewer window("surface subdivision", 1200, 600);
    window.run();
}

