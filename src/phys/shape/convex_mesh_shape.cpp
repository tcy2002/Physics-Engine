#include "convex_mesh_shape.h"
#include <algorithm>

namespace pe_phys_shape {

    ConvexMeshShape::ConvexMeshShape(pe::Mesh mesh):
        _mesh(std::move(mesh)) {}

    void ConvexMeshShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        min = PE_VEC_MAX;
        max = PE_VEC_MIN;
        auto &rot = transform.getBasis();
        auto &pos = transform.getOrigin();
        for (auto &p: _mesh.vertices) {
            auto v = rot * p.position;
            min = PE_MIN_VEC(min, v);
            max = PE_MAX_VEC(max, v);
        }
        min += pos;
        max += pos;
    }

    bool ConvexMeshShape::isInside(const pe::Transform &transform, const pe::Vector3 &point) const {
        auto local_point = transform.inverseTransform(point);
        return !std::any_of(_mesh.faces.begin(), _mesh.faces.end(), [&](auto &f) {
            auto &normal = f.normal;
            auto &p0 = _mesh.vertices[f.indices[0]].position;
            return normal.dot(local_point - p0) > 0;
        });
    }

    void ConvexMeshShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                  pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        pe::Matrix3 rot = transform.getBasis();
        pe::Vector3 trans = transform.getOrigin();
        pe::Vector3 local_axis = rot.transposed() * axis;
        pe::Real offset = trans.dot(axis);

        minProj = PE_REAL_MAX;
        maxProj = PE_REAL_MIN;
        for (auto &p: _mesh.vertices) {
            auto v = p.position.dot(local_axis);
            minProj = std::min(minProj, v);
            maxProj = std::max(maxProj, v);
        }

        minProj += offset;
        maxProj += offset;
        minPoint = axis * minProj;
        maxPoint = axis * maxProj;
    }

}