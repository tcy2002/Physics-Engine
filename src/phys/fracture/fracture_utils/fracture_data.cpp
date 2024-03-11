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
        auto x = (uint32_t)(std::round(v.x / PE_EPS));
        auto y = (uint32_t)(std::round(v.y / PE_EPS));
        auto z = (uint32_t)(std::round(v.z / PE_EPS));
        uint32_t h = 0x995af;
        h ^= x + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= y + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= z + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }

    bool vertex::equal(const vertex& a, const vertex& b) {
        return approx_equal(a.pos, b.pos) && approx_equal(a.nor, b.nor);
    }

    uint32_t vertex::hash_func(const vertex& v) {
        auto x = vector3_hash_func(v.pos);
        auto y = vector3_hash_func(v.nor);
        uint32_t h = 0x10f1;
        h ^= x + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= y + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }

    bool triangle::equal(const triangle& a, const triangle& b) {
        return (a.vert_ids[0] == b.vert_ids[0] || a.vert_ids[0] == b.vert_ids[1] || a.vert_ids[0] == b.vert_ids[2]) &&
               (a.vert_ids[1] == b.vert_ids[0] || a.vert_ids[1] == b.vert_ids[1] || a.vert_ids[1] == b.vert_ids[2]) &&
               (a.vert_ids[2] == b.vert_ids[0] || a.vert_ids[2] == b.vert_ids[1] || a.vert_ids[2] == b.vert_ids[2]);
    }

    uint32_t triangle::hash_func(const triangle& t) {
        auto id1 = t.vert_ids[0], id2 = t.vert_ids[1], id3 = t.vert_ids[2];
        sort3(id1, id2, id3);
        uint32_t h = 0x301eef;
        h ^= (id1 + 0x9e3779b9) + (h << 6) + (h >> 2);
        h ^= (id2 + 0x9e3779b9) + (h << 6) + (h >> 2);
        h ^= (id3 + 0x9e3779b9) + (h << 6) + (h >> 2);
        return h;
    }

    bool polygon::equal(const polygon& a, const polygon& b) {
        return approx_equal(a.nor, b.nor);
    }

    bool polygon::hash_func(const polygon& p) {
        return vector3_hash_func(p.nor);
    }

} // namespace pe_phys_fracture