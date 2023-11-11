#pragma once

#include "def.h"
#include "vector3.h"

namespace pe_cg {

class Mesh {
public:
    struct Vertex {
        Vector3 position;
        Vector3 normal;
    };
    struct Face {
        pe::Array<uint32_t> indices;
        Vector3 normal;
    };

    pe::Array<Vertex> vertices;
    pe::Array<Face> faces;

    Mesh() = default;
    Mesh(pe::Array<Vertex> vs, pe::Array<Face> fs):
    vertices(std::move(vs)), faces(std::move(fs)) {}
};

} // namespace data_cg