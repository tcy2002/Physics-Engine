#include "convex_mesh_shape.h"
#include <algorithm>

using namespace pe_phys_shape;

ConvexMeshShape::ConvexMeshShape(pe_cg::Mesh mesh) {
    _mesh = std::move(mesh);
}

ShapeType ConvexMeshShape::getType() const {
    return ShapeType::MESH;
}

bool ConvexMeshShape::isConvex() const {
    return true;
}

void ConvexMeshShape::getAABB(const pe_cg::Transform& transform, pe_cg::Vector3& min, pe_cg::Vector3& max) const {
    min = VEC_MAX;
    max = VEC_MIN;
    auto& rot = transform.getBasis();
    auto& pos = transform.getOrigin();
    for (auto& p: _mesh.vertices) {
        auto v = rot * p.position;
        min = MIN_VEC(min, v);
        max = MAX_VEC(max, v);
    }
    min += pos;
    max += pos;
}

bool ConvexMeshShape::isInside(const pe_cg::Transform& transform, const pe_cg::Vector3& point) const {
    auto local_point = transform.inverseTransform(point);
    return !std::any_of(_mesh.faces.begin(), _mesh.faces.end(), [&](auto& f) {
        auto& normal = f.normal;
        auto& p0 = _mesh.vertices[f.indices[0]].position;
        return normal.dot(local_point - p0) > 0;
    });
}

void ConvexMeshShape::project(const pe_cg::Transform& transform, const pe_cg::Vector3& axis, real& min, real& max) const {
    auto local_axis = transform.getBasis().transposed() * axis;
    auto offset = transform.getOrigin().dot(axis);
    min = REAL_MAX;
    max = REAL_MIN;
    for (auto& p: _mesh.vertices) {
        auto v = p.position.dot(local_axis);
        min = std::min(min, v);
        max = std::max(max, v);
    }
    min += offset;
    max += offset;
}
