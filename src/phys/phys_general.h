#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <map>
#include <cstdint>
#include "utils/hash_vector.h"

//// general macros
#include <common/general.h>

//// real
#ifdef PE_USE_DOUBLE
#define PE_REAL_MAX 1e100
#define PE_REAL_MIN -1e100
#else
#define PE_REAL_MAX 1e100
#define PE_REAL_MIN -1e100
#endif

//// min max
#define PE_MAX(a, b) ((a) > (b) ? (a) : (b))
#define PE_MIN(a, b) ((a) < (b) ? (a) : (b))
#define PE_MAX3(a, b, c) PE_MAX(PE_MAX(a, b), c)
#define PE_MIN3(a, b, c) PE_MIN(PE_MIN(a, b), c)

//// geometry types
#include <common/mesh.h>
#include <common/vector3.h>
#include <common/matrix3x3.h>
#include <common/transform.h>
namespace pe {
#ifdef PE_USE_DOUBLE
    using Real = double;
#else
    using Real = float;
#endif
    using Mesh = common::Mesh<Real>;
    using Vector3 = common::Vector3<Real>;
    using Matrix3 = common::Matrix3x3<Real>;
    using Transform = common::Transform<Real>;
} // namespace pe

//// vector3
#define PE_VEC_MAX pe::Vector3(PE_REAL_MAX, PE_REAL_MAX, PE_REAL_MAX)
#define PE_VEC_MIN pe::Vector3(PE_REAL_MIN, PE_REAL_MIN, PE_REAL_MIN)

//// math
#define PE_EPS pe::Real(0.00001)
#define PE_APPROX_EQUAL(a, b) (std::abs((a) - (b)) < PE_EPS)
#define PE_PI pe::Real(3.141592653589)

//// other date types
namespace pe {
    template <typename T>
    using Array = std::vector<T>;

    template <typename K, typename V>
    using Map = std::map<K, V>;

    template <typename K, typename V>
    using HashMap = std::unordered_map<K, V>;

    template <typename T, typename HashFunc, typename EqualFunc>
    using HashList = utils::hash_vector<T, HashFunc, EqualFunc>;

    template <typename T1, typename T2>
    using KV = std::pair<T1, T2>;

    using Uint32HashList = HashList<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>>;

    struct Vector3Equal {
        bool operator()(const pe::Vector3& a, const pe::Vector3& b) const {
            return PE_APPROX_EQUAL(a.x, b.x) && PE_APPROX_EQUAL(a.y, b.y) && PE_APPROX_EQUAL(a.z, b.z);
        }
    };
    struct Vector3Hash {
        uint32_t operator()(const pe::Vector3& v) const {
            auto x = (uint32_t)(std::round(v.x / PE_EPS));
            auto y = (uint32_t)(std::round(v.y / PE_EPS));
            auto z = (uint32_t)(std::round(v.z / PE_EPS));
            uint32_t h = 0x995af;
            h ^= x + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= y + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= z + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };
    using Vector3HashList = HashList<Vector3, Vector3Hash, Vector3Equal>;
} // namespace pe