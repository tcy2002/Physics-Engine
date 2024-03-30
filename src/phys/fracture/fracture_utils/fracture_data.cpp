#include "fracture_data.h"

namespace pe_phys_fracture {

    uint32_t vertex::VertexHash::operator()(const vertex& v) const {
        static pe::Vector3Hash hash;
        auto x = hash(v.pos);
        auto y = hash(v.nor);
        uint32_t h = 0x10f1;
        h ^= x + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= y + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }

    bool vertex::VertexEqual::operator()(const vertex& a, const vertex& b) const {
        static pe::Vector3Equal equal;
        return equal(a.pos, b.pos) && equal(a.nor, b.nor);
    }

    uint32_t triangle::TriangleHash::operator()(const triangle& t) const {
        auto id1 = t.vert_ids[0], id2 = t.vert_ids[1], id3 = t.vert_ids[2];
        sort3(id1, id2, id3);
        uint32_t h = 0x301eef;
        h ^= (id1 + 0x9e3779b9) + (h << 6) + (h >> 2);
        h ^= (id2 + 0x9e3779b9) + (h << 6) + (h >> 2);
        h ^= (id3 + 0x9e3779b9) + (h << 6) + (h >> 2);
        return h;
    }

    bool triangle::TriangleEqual::operator()(const triangle& a, const triangle& b) const {
        return (a.vert_ids[0] == b.vert_ids[0] || a.vert_ids[0] == b.vert_ids[1] || a.vert_ids[0] == b.vert_ids[2]) &&
               (a.vert_ids[1] == b.vert_ids[0] || a.vert_ids[1] == b.vert_ids[1] || a.vert_ids[1] == b.vert_ids[2]) &&
               (a.vert_ids[2] == b.vert_ids[0] || a.vert_ids[2] == b.vert_ids[1] || a.vert_ids[2] == b.vert_ids[2]);
    }

    void polygon::add_vert(uint32_t v, uint32_t idx) {
        if (idx == -1) {
            vert_ids.push_back(v);
        }
        else {
            vert_ids.insert(vert_ids.begin() + idx, v);
        }
    }

    bool polygon::remove_vert(uint32_t idx) {
        if (idx >= vert_ids.size()) {
            return false;
        }
        vert_ids.erase(vert_ids.begin() + idx);
        return true;
    }

    uint32_t polygon::PolygonHash::operator()(const polygon& p) const {
        static pe::Vector3Hash hash;
        return hash(p.nor);
    }

    bool polygon::PolygonEqual::operator()(const polygon& a, const polygon& b) const {
        static pe::Vector3Equal equal;
        return equal(a.nor, b.nor);
    }

} // namespace pe_phys_fracture