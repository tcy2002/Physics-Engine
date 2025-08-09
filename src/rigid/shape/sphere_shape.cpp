#include "sphere_shape.h"

namespace pe_phys_shape {

    SphereShape::SphereShape(pe::Real radius): _radius(radius) {
        _volume = PE_R(4.0 / 3.0) * PE_PI * radius * radius * radius;
        const pe::Real i = PE_R(2.0 / 5.0) * _radius * _radius;
        _local_inertia <<
            i, 0, 0,
            0, i, 0,
            0, 0, i;
    }

    void SphereShape::getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const {
        const pe::Vector3& center = transform.getOrigin();
        min = center - pe::Vector3(_radius, _radius, _radius);
        max = center + pe::Vector3(_radius, _radius, _radius);
    }

    bool SphereShape::localIsInside(const pe::Vector3 &point) const {
        return point.squaredNorm() <= _radius * _radius;
    }

    void SphereShape::project(const pe::Transform& transform, const pe::Vector3& axis, pe::Real& minProj,
                           pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        const pe::Vector3& trans = transform.getOrigin();
        const pe::Real offset = trans.dot(axis);
        minProj = offset - _radius;
        maxProj = offset + _radius;
        minPoint = trans - axis * _radius;
        maxPoint = trans + axis * _radius;
    }

}