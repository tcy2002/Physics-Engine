#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <map>
#include <cstdint>

//// dll export
// you should mark PE_API for public functions that you
// want to use in demos or other projects
#ifdef _MSC_VER
#define PE_API _declspec(dllexport)
#else
#define PE_API
#endif

//// general macros
#include <common/general.h>

//// data
#define PE_DATA_DOWNLOAD_PATH "./data"

//// type cast
#define R(n) static_cast<pe::Real>(n)
#define F(n) static_cast<float>(n)
#define D(n) static_cast<double>(n)
#define UI(n) static_cast<uint32_t>(n)
#define I(n) static_cast<int>(n)
#define IL(n) static_cast<int64_t>(n)
#define UIL(n) static_cast<uint64_t>(n)

//// real
#ifdef PE_USE_DOUBLE
#define PE_REAL_MAX R(1e100)
#define PE_REAL_MIN R(-1e100)
#else
#define PE_REAL_MAX R(1e30)
#define PE_REAL_MIN R(-1e30)
#endif

//// phys
#define PE_MARGIN pe::Real(0.005)
#define PE_DIST_REF_RADIO pe::Real(0.05)
#define PE_DIST_TH pe::Real(0.02)
#define PE_USE_QUATERNION

//// math
#define PE_SWAP(a, b) { auto t = a; a = b; b = t; }
#define PE_MAX(a, b) ((a) > (b) ? (a) : (b))
#define PE_MIN(a, b) ((a) < (b) ? (a) : (b))
#define PE_MAX3(a, b, c) PE_MAX(PE_MAX(a, b), c)
#define PE_MIN3(a, b, c) PE_MIN(PE_MIN(a, b), c)
#define PE_CLAMP(x, min, max) PE_MAX(PE_MIN(x, max), min)
#define PE_SQR(x) ((x) * (x))
#define PE_POW(x, n) std::pow(x, n)
#define PE_SQRT(x) std::sqrt(x)
#define PE_ABS(x) std::abs(x)
#define PE_SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#define PE_LERP(a, b, t) ((a) + (t) * ((b) - (a)))
#define PE_DEG_TO_RAD(x) ((x) * PE_PI / 180)
#define PE_RAD_TO_DEG(x) ((x) * 180 / PE_PI)
#define PE_COS std::cos
#define PE_SIN std::sin
#define PE_TAN std::tan
#define PE_ACOS std::acos
#define PE_ASIN std::asin
#define PE_ATAN2 std::atan2

//// geometry types
#include <common/mesh.h>
#include <common/vector3.h>
#include <common/matrix3x3.h>
#include <common/transform.h>
#include <common/quaternion.h>
#include "utils/vector_x.h"
#include "utils/matrix_m_n.h"
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
    using Quaternion = common::Quaternion<Real>;
    using VectorX = utils::VectorX<Real>;
    using MatrixMN = utils::MatrixMN<Real>;
} // namespace pe

//// vector3
#define PE_VEC_MAX pe::Vector3(PE_REAL_MAX, PE_REAL_MAX, PE_REAL_MAX)
#define PE_VEC_MIN pe::Vector3(PE_REAL_MIN, PE_REAL_MIN, PE_REAL_MIN)

//// math
#define PE_EPS pe::Real(0.00001)
#define PE_APPROX_EQUAL(a, b) (std::abs((a) - (b)) < PE_EPS)
#define PE_PI pe::Real(3.141592653589)

//// other date types
#include "utils/hash_vector.h"
namespace pe {
    template <typename T> using Array = std::vector<T>;
    template <typename T> using Queue = std::queue<T>;
    template <typename K, typename V> using Map = std::map<K, V>;
    template <typename V> using Set = std::set<V>;
    template <typename V> using HashSet = std::unordered_set<V>;
    template <typename K, typename V> using HashMap = std::unordered_map<K, V>;
    template <typename T, typename HashFunc, typename EqualFunc> using HashList = utils::hash_vector<T, HashFunc, EqualFunc>;
    template <typename T1, typename T2> using KV = std::pair<T1, T2>;

    using Uint32HashList = HashList<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>>;

    struct Vector3Equal {
        bool operator()(const pe::Vector3& a, const pe::Vector3& b) const {
            return PE_APPROX_EQUAL(a.x, b.x) && PE_APPROX_EQUAL(a.y, b.y) && PE_APPROX_EQUAL(a.z, b.z);
        }
    };
    struct Vector3Hash {
        uint32_t operator()(const pe::Vector3& v) const {
            auto x = UI(std::round(v.x / PE_EPS));
            auto y = UI(std::round(v.y / PE_EPS));
            auto z = UI(std::round(v.z / PE_EPS));
            uint32_t h = 0x995af;
            h ^= x + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= y + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= z + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };
    using Vector3HashList = HashList<Vector3, Vector3Hash, Vector3Equal>;
} // namespace pe