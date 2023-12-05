#pragma once

#include "common/mesh.h"
#include "common/vector3.h"
#include "common/matrix3x3.h"
#include "common/transform.h"

namespace simple_viewer {

    //// input data types for simple viewer
#ifdef PE_USE_DOUBLE
    using Mesh = common::Mesh<double>;
    using Vector3 = common::Vector3<double>;
    using Matrix3 = common::Matrix3x3<double>;
    using Transform = common::Transform<double>;
#else
    using Mesh = common::Mesh<float>;
    using Vector3 = common::Vector3<float>;
    using Matrix3 = common::Matrix3x3<float>;
    using Transform = common::Transform<float>;
#endif

} // namespace viewer