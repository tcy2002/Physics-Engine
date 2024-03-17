#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <map>

//// general macros
#include "common/general.h"

//// real
#ifdef PE_USE_DOUBLE
#define PE_REAL_MAX 1e100
#define PE_REAL_MIN -1e100
#else
#define PE_REAL_MAX 1e100
#define PE_REAL_MIN -1e100
#endif

//// min max
#ifdef _MSC_VER
#define PE_MAX(a, b) max(a, b)
#define PE_MIN(a, b) min(a, b)
#else
#define PE_MAX(a, b) std::max(a, b)
#define PE_MIN(a, b) std::min(a, b)
#endif

//// geometry types
#include "common/mesh.h"
#include "common/vector3.h"
#include "common/matrix3x3.h"
#include "common/transform.h"
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

//// vector macros
#define PE_MAX_VEC(a, b) pe::Vector3(PE_MAX((a).x, (b).x), PE_MAX((a).y, (b).y), PE_MAX((a).z, (b).z))
#define PE_MIN_VEC(a, b) pe::Vector3(PE_MIN((a).x, (b).x), PE_MIN((a).y, (b).y), PE_MIN((a).z, (b).z))
#define PE_VEC_MAX pe::Vector3(PE_REAL_MAX, PE_REAL_MAX, PE_REAL_MAX)
#define PE_VEC_MIN pe::Vector3(PE_REAL_MIN, PE_REAL_MIN, PE_REAL_MIN)

//// math
#define PE_EPS 1e-5
#define PE_APPROX_EQUAL(a, b) (std::abs((a) - (b)) < PE_EPS)
#define PE_PI 3.14159265358979323846

//// other date types
namespace pe {
    template <typename T> using Array = std::vector<T>;
    template <typename T> using Queue = std::queue<T>;
    template <typename K, typename V> using Map = std::map<K, V>;
    template <typename K, typename V> using HashMap = std::unordered_map<K, V>;
    template <typename K, typename V> using MultiMap = std::unordered_multimap<K, V>;
    template <typename T1, typename T2> using KV = std::pair<T1, T2>;
} // namespace pe