#pragma once

#include <vector>
#include <queue>
#include <unordered_map>

//// general macros
#include "common/general.h"

//// real
#ifdef PE_USE_DOUBLE
#define PE_REAL_MAX __DBL_MAX__
#define PE_REAL_MIN __DBL_DENORM_MIN__
#else
#define PE_REAL_MAX __FLT_MAX__
#define PE_REAL_MIN __FLT_DENORM_MIN__
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

//// error tolerance
#define PE_EPS 1e-5
#define PE_APPROX_EQUAL(a, b) (std::abs((a) - (b)) < PE_EPS)

//// other date types
namespace pe {
    template <typename T> using Array = std::vector<T>;
    template <typename T> using Queue = std::queue<T>;
    template <typename K, typename V> using KVStore = std::unordered_map<K, V>;
    template <typename T1, typename T2> using Pair = std::pair<T1, T2>;
} // namespace pe
