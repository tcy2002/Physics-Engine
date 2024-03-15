#include "cylinder_shape.h"

namespace pe_phys_shape {

    CylinderShape::CylinderShape(pe::Real radius, pe::Real height): _radius(radius), _height(height) {}

    void CylinderShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        pe::Vector3 axis = transform.getBasis().getColumn(1);
        pe::Vector3 extent = axis.getAbsolute() * (_height * 0.5);
        extent.y += _radius * std::sqrt(axis.x * axis.x + axis.z * axis.z);
        extent.x += _radius * std::sqrt(axis.y * axis.y + axis.z * axis.z);
        extent.z += _radius * std::sqrt(axis.x * axis.x + axis.y * axis.y);
        pe::Vector3 center = transform.getOrigin();
        min = center - extent;
        max = center + extent;
    }

    bool CylinderShape::localIsInside(const pe::Vector3 &point, pe::Real margin) const {
        return point.y >= -_height * 0.5 - margin && point.y <= _height * 0.5 + margin &&
               point.x * point.x + point.z * point.z <= _radius * _radius + margin * margin;
    }

    void CylinderShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        pe::Vector3 local_axis = transform.getBasis().transposed() * axis;
        pe::Real half_height = _height * 0.5;
        pe::Real offset = transform.getOrigin().dot(axis);
        if (PE_APPROX_EQUAL(local_axis.x, 0) && PE_APPROX_EQUAL(local_axis.z, 0)) {
            pe::Vector3 up = pe::Vector3::up() * half_height;
            minPoint = transform * -up;
            maxPoint = transform * up;
            minProj = offset - half_height;
            maxProj = offset + half_height;
        } else if (PE_APPROX_EQUAL(local_axis.y, 0)) {
            pe::Vector3 r = local_axis * _radius;
            minPoint = transform * -r;
            maxPoint = transform * r;
            minProj = offset - _radius;
            maxProj = offset + _radius;
        } else {
            pe::Real r = std::sqrt(local_axis.x * local_axis.x + local_axis.z * local_axis.z);
            pe::Real x = _radius * local_axis.x / r;
            pe::Real z = _radius * local_axis.z / r;
            pe::Real y = half_height * (local_axis.y > 0 ? 1 : -1);
            pe::Vector3 ext = pe::Vector3(x, y, z);
            minPoint = transform * -ext;
            maxPoint = transform * ext;
            pe::Real ext_l = ext.dot(local_axis);
            minProj = offset - ext_l;
            maxProj = offset + ext_l;
        }
    }

    pe::Matrix3 CylinderShape::calcLocalInertia(pe::Real mass) const {
        pe::Real r2 = _radius * _radius;
        pe::Real h2 = _height * _height;
        pe::Real axis = mass * (r2 * 0.5);
        pe::Real diag = mass * (r2 / 4.0 + h2 / 12.0);
        return {
                diag, 0, 0,
                0, axis, 0,
                0, 0, diag
        };
    }

}