#pragma once

#include <cstdint>
#include "phys/phys_general.h"
#include "fracture_utils.h"

namespace pe_phys_fracture {

    struct vertex {
        pe::Vector3 pos;
        pe::Vector3 nor;
        vertex() = default;
        vertex(const pe::Vector3& p, const pe::Vector3& n) : pos(p), nor(n) {}
        struct VertexHash { uint32_t operator()(const vertex& v) const; };
        struct VertexEqual { bool operator()(const vertex& a, const vertex& b) const; };
    };
    using VertexHashList = pe::HashList<vertex, vertex::VertexHash, vertex::VertexEqual>;

    struct triangle {
        uint32_t vert_ids[3]{};
        triangle() = default;
        triangle(uint32_t v1, uint32_t v2, uint32_t v3) {
            vert_ids[0] = v1;
            vert_ids[1] = v2;
            vert_ids[2] = v3;
        }
        struct TriangleHash { uint32_t operator()(const triangle& t) const; };
        struct TriangleEqual { bool operator()(const triangle& a, const triangle& b) const; };
    };
    using TriangleHashList = pe::HashList<triangle, triangle::TriangleHash, triangle::TriangleEqual>;

    struct polygon {
        pe::Array<uint32_t> vert_ids;
        pe::Vector3 nor;
        polygon() = default;
        explicit polygon(const pe::Vector3& n) : nor(n) {}
        void add_vert(uint32_t v, uint32_t idx = -1);
        bool remove_vert(uint32_t idx);
        struct PolygonHash { uint32_t operator()(const polygon& p) const; };
        struct PolygonEqual { bool operator()(const polygon& a, const polygon& b) const; };
    };
    using PolygonHashList = pe::HashList<polygon, polygon::PolygonHash, polygon::PolygonEqual>;

    struct tetrahedron {
        uint32_t tri_ids[4];
        pe::Vector3 center;
        pe::Real radius;
        tetrahedron() : radius(0) {}
        tetrahedron(uint32_t t1, uint32_t t2, uint32_t t3, uint32_t t4, const pe::Vector3& c, pe::Real r):
                    center(c), radius(r), tri_ids{t1, t2, t3, t4} {}
    };

} // namespace pe_phys_fracture