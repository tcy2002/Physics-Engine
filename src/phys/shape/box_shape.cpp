#include "box_shape.h"

namespace pe_phys_shape {

    BoxShape::BoxShape(const pe::Vector3 &size):
        _size(size), _half_size(size / 2) {}

    pe::Vector3 BoxShape::localGetSupportVertex(const pe::Vector3 &dir) const {
        return pe::Vector3(
                dir.x > 0 ? _half_size.x : -_half_size.x,
                dir.y > 0 ? _half_size.y : -_half_size.y,
                dir.z > 0 ? _half_size.z : -_half_size.z
        );
    }

    void BoxShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        min = PE_VEC_MAX;
        max = PE_VEC_MIN;
        auto &rot = transform.getBasis();
        auto &pos = transform.getOrigin();
        pe::Vector3 p;
        for (int i = 0; i < 8; i++) {
            p.x = i & 1 ? _half_size.x : -_half_size.x;
            p.y = i & 2 ? _half_size.y : -_half_size.y;
            p.z = i & 4 ? _half_size.z : -_half_size.z;
            auto v = rot * p;
            min = PE_MIN_VEC(min, v);
            max = PE_MAX_VEC(max, v);
        }
        min += pos;
        max += pos;
    }

    bool BoxShape::localIsInside(const pe::Vector3 &point) const {
        return std::abs(point.x) <= _half_size.x &&
               std::abs(point.y) <= _half_size.y &&
               std::abs(point.z) <= _half_size.z;
    }

    void BoxShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                           pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        pe::Matrix3 rot = transform.getBasis();
        pe::Vector3 trans = transform.getOrigin();
        pe::Vector3 local_axis = rot.transposed() * axis;
        pe::Real offset = trans.dot(axis);

        pe::Vector3 ext;
        ext.x = local_axis.x > 0 ? _half_size.x : -_half_size.x;
        ext.y = local_axis.y > 0 ? _half_size.y : -_half_size.y;
        ext.z = local_axis.z > 0 ? _half_size.z : -_half_size.z;
        pe::Real half_ext = ext.dot(local_axis);

        minProj = offset - half_ext;
        maxProj = offset + half_ext;
        minPoint = axis * minProj;
        maxPoint = axis * maxProj;
    }

}