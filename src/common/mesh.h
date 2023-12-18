#pragma once

#include "vector3.h"
#include <vector>

namespace common {

    template <typename Scalar>
    class Mesh {
    public:
        struct Vertex {
            Vector3<Scalar> position;
            Vector3<Scalar> normal;
        };
        struct Face {
            std::vector<uint32_t> indices;
            Vector3<Scalar> normal;
        };

        std::vector<Vertex> vertices;
        std::vector<Face> faces;

        Mesh() = default;
        Mesh(std::vector<Vertex> vs, std::vector<Face> fs):
            vertices(std::move(vs)), faces(std::move(fs)) {}
    };

} // namespace common