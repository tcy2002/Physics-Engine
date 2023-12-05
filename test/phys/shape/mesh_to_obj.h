#pragma once

#include <fstream>
#include "phys/public_include.h"

void meshToObj(const pe::Mesh &mesh, const std::string &filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return;
    }

    for (auto& point : mesh.vertices) {
        auto& p = point.position;
        auto& n = point.normal;
        file << "v " << p.x << " " << p.y << " " << p.z << std::endl;
        file << "vn " << n.x << " " << n.y << " " << n.z << std::endl;
    }
    for (auto& face: mesh.faces) {
        file << "f";
        for (auto& index : face.indices) {
            file << " " << index + 1 << "//" << index + 1;
        }
        file << std::endl;
    }
    file.close();
}
