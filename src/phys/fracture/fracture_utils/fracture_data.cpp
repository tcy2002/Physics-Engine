#include "fracture_data.h"

namespace pe_phys_fracture {

    bool uint32_equal(uint32_t a, uint32_t b) {
        return a == b;
    }

    uint32_t uint32_hash_func(uint32_t v) {
        return v;
    }

    bool vector3_equal(const pe::Vector3& a, const pe::Vector3& b) {
        return approx_equal(a, b);
    }

    uint32_t vector3_hash_func(const pe::Vector3& v) {
        auto x = (uint32_t)std::round(v.x / PE_EPS);
        auto y = (uint32_t)std::round(v.y / PE_EPS);
        auto z = (uint32_t)std::round(v.z / PE_EPS);
        return (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
    }

    bool vertex::equal(const vertex& a, const vertex& b) {
        return approx_equal(a.pos, b.pos) && approx_equal(a.nor, b.nor);
    }

    uint32_t vertex::hash_func(const vertex& v) {
        return vector3_hash_func(v.pos) ^ vector3_hash_func(v.nor) * 73856093;
    }

    bool triangle::equal(const triangle& a, const triangle& b) {
        return (a.vert_ids[0] == b.vert_ids[0] || a.vert_ids[0] == b.vert_ids[1] || a.vert_ids[0] == b.vert_ids[2]) &&
               (a.vert_ids[1] == b.vert_ids[0] || a.vert_ids[1] == b.vert_ids[1] || a.vert_ids[1] == b.vert_ids[2]) &&
               (a.vert_ids[2] == b.vert_ids[0] || a.vert_ids[2] == b.vert_ids[1] || a.vert_ids[2] == b.vert_ids[2]);
    }

    uint32_t triangle::hash_func(const triangle& t) {
        auto id1 = t.vert_ids[0], id2 = t.vert_ids[1], id3 = t.vert_ids[2];
        sort3(id1, id2, id3);
        return (id1 * 73856093) ^ (id2 * 19349663) ^ (id3 * 83492791);
    }

    bool polygon::equal(const polygon& a, const polygon& b) {
        return approx_equal(a.nor, b.nor);
    }

    bool polygon::hash_func(const polygon& p) {
        return vector3_hash_func(p.nor);
    }

} // namespace pe_phys_fracture