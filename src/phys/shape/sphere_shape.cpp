#include "sphere_shape.h"

namespace pe_phys_shape {

    SphereShape::SphereShape(pe::Real radius): _radius(radius) {}

    void SphereShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        pe::Vector3 center = transform.getOrigin();
        min = center - pe::Vector3(_radius, _radius, _radius);
        max = center + pe::Vector3(_radius, _radius, _radius);
    }

    bool SphereShape::localIsInside(const pe::Vector3 &point) const {
        return point.norm2() <= _radius * _radius;
    }

    void SphereShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                           pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        pe::Vector3 trans = transform.getOrigin();
        pe::Real offset = trans.dot(axis);
        minProj = offset - _radius;
        maxProj = offset + _radius;
        minPoint = trans - axis * _radius;
        maxPoint = trans + axis * _radius;
    }

    pe::Matrix3 SphereShape::calcLocalInertia(pe::Real mass) const {
        pe::Real i = pe::Real(2.0) / pe::Real(5.0) * mass * _radius * _radius;
        return {
                i, 0, 0,
                0, i, 0,
                0, 0, i
        };
    }

}