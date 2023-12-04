#pragma once

#include "phys/def.h"
#include "vector3.h"

namespace common {

template <typename Scalar>
class Mesh {
public:
    struct Vertex {
        Vector3<Scalar> position;
        Vector3<Scalar> normal;
    };
    struct Face {
        pe::Array<uint32_t> indices;
        Vector3<Scalar> normal;
    };

    pe::Array<Vertex> vertices;
    pe::Array<Face> faces;

    Mesh() = default;
    Mesh(pe::Array<Vertex> vs, pe::Array<Face> fs):
        vertices(std::move(vs)), faces(std::move(fs)) {}
};

} // namespace common