#pragma once

#include "common/mesh.h"
#include "common/vector3.h"
#include "common/matrix3x3.h"
#include "common/transform.h"

namespace simple_viewer {
    using Vector3 = common::Vector3<float>;
    using Matrix3 = common::Matrix3x3<float>;
    using Transform = common::Transform<float>;
#ifdef PE_USE_DOUBLE
    using Mesh = common::Mesh<double>;
#else
    using Mesh = common::Mesh<float>;
#endif
} // namespace viewer