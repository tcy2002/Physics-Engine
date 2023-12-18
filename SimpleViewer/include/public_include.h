#pragma once

#include "common/mesh.h"
#include "common/vector3.h"
#include "common/matrix3x3.h"
#include "common/transform.h"

#ifdef _MSC_VER
#define SV_API _declspec(dllexport)
#else
#define SV_API
#endif

namespace simple_viewer {

    using Mesh = common::Mesh<float>;
    using Vector3 = common::Vector3<float>;
    using Matrix3 = common::Matrix3x3<float>;
    using Transform = common::Transform<float>;

    // calculate per-face-normal
    SV_API void per_face_normal(Mesh& mesh);

    // calculate per-vertex-normal
    SV_API void per_vertex_normal(Mesh& mesh);

} // namespace viewer