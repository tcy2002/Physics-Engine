#include "concave_mesh_shape.h"
#include <algorithm>
#include "phys/fracture/fracture_utils/fracture_utils.h"

namespace pe_phys_shape {

    void ConcaveMeshShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        // the same as ConvexMeshShape
        min = PE_VEC_MAX;
        max = PE_VEC_MIN;
        auto &rot = transform.getBasis();
        auto &pos = transform.getOrigin();
        for (auto &p: _mesh.vertices) {
            auto v = rot * p.position;
            min = pe::Vector3::min2(min, v);
            max = pe::Vector3::max2(max, v);
        }
        min += pos;
        max += pos;
    }

    bool ConcaveMeshShape::localIsInside(const pe::Vector3 &point) const {
        // not implemented
        return false;
    }

    void ConcaveMeshShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                  pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        // the same as ConvexMeshShape
        pe::Matrix3 rot = transform.getBasis();
        pe::Vector3 trans = transform.getOrigin();
        pe::Vector3 local_axis = rot.transposed() * axis;
        pe::Real offset = trans.dot(axis);

        minProj = PE_REAL_MAX;
        maxProj = PE_REAL_MIN;
        for (auto &p: _mesh.vertices) {
            auto v = p.position.dot(local_axis);
            if (v < minProj) {
                minProj = v;
                minPoint = p.position;
            } else if (v > maxProj) {
                maxProj = v;
                maxPoint = p.position;
            }
        }

        minProj += offset;
        maxProj += offset;
        minPoint = transform * minPoint;
        maxPoint = transform * maxPoint;
    }

    pe::Matrix3 ConcaveMeshShape::calcLocalInertia(pe::Real mass) const {
        // not implemented
        return pe::Matrix3::identity();
    }
}