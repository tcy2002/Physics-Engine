#pragma once

#include "general.h"
#include <vector>
#include <iostream>
#include <fstream>

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

        Mesh() {}
        Mesh(const std::vector<Vertex>& vs, const std::vector<Face>& fs):
            vertices(vs), faces(fs) {}
        Mesh(std::vector<Vertex>&& vs, std::vector<Face>&& fs):
            vertices(std::move(vs)), faces(std::move(fs)) {}

        bool empty() const { return vertices.empty() || faces.empty(); }

        static void perFaceNormal(Mesh& mesh);
        static void perVertexNormal(Mesh& mesh);
        static void loadFromObj(const std::string &filename, Mesh<Scalar> &mesh, const Vector3<Scalar>& size);
        static void saveToObj(const std::string &filename, const Mesh<Scalar> &mesh, const Vector3<Scalar>& size);
    };

    #include "mesh.cpp"

} // namespace common