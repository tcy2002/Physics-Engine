#include "convex_mesh_shape.h"
#include <algorithm>

using namespace pe_phys_shape;

ConvexMeshShape::ConvexMeshShape(pe_common::Mesh mesh) {
    _mesh = std::move(mesh);
}

ShapeType ConvexMeshShape::getType() const {
    return ShapeType::MESH;
}

bool ConvexMeshShape::isConvex() const {
    return true;
}

void ConvexMeshShape::getAABB(const pe_common::Transform& transform, pe_common::Vector3& min, pe_common::Vector3& max) const {
    min = PE_VEC_MAX;
    max = PE_VEC_MIN;
    auto& rot = transform.getBasis();
    auto& pos = transform.getOrigin();
    for (auto& p: _mesh.vertices) {
        auto v = rot * p.position;
        min = PE_MIN_VEC(min, v);
        max = PE_MAX_VEC(max, v);
    }
    min += pos;
    max += pos;
}

bool ConvexMeshShape::isInside(const pe_common::Transform& transform, const pe_common::Vector3& point) const {
    auto local_point = transform.inverseTransform(point);
    return !std::any_of(_mesh.faces.begin(), _mesh.faces.end(), [&](auto& f) {
        auto& normal = f.normal;
        auto& p0 = _mesh.vertices[f.indices[0]].position;
        return normal.dot(local_point - p0) > 0;
    });
}

void ConvexMeshShape::project(const pe_common::Transform& transform, const pe_common::Vector3& axis, PEReal& min, PEReal& max) const {
    auto local_axis = transform.getBasis().transposed() * axis;
    auto offset = transform.getOrigin().dot(axis);
    min = PE_REAL_MAX;
    max = PE_REAL_MIN;
    for (auto& p: _mesh.vertices) {
        auto v = p.position.dot(local_axis);
        min = std::min(min, v);
        max = std::max(max, v);
    }
    min += offset;
    max += offset;
}
