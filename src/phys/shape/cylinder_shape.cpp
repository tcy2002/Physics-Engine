#include "cylinder_shape.h"
#include "default_mesh.h"

namespace pe_phys_shape {

    CylinderShape::CylinderShape(pe::Real radius, pe::Real height): _radius(radius), _height(height) {
        _mesh = PE_CYLINDER_DEFAULT_MESH;
        pe::Real r2 = radius * 2;
        for (auto& v : _mesh.vertices) {
            v.position.x *= r2;
            v.position.z *= r2;
            v.position.y *= height;
        }
        _unique_edges = _cylinder_unique_edges;
        for (auto& edge : _unique_edges) {
            edge.first.x *= r2;
            edge.first.z *= r2;
            edge.first.y *= height;
            edge.second.x *= r2;
            edge.second.z *= r2;
            edge.second.y *= height;
        }
        pe::Real r_2 = _radius * _radius;
        pe::Real h_2 = _height * _height;
        pe::Real axis = r_2 * pe::Real(0.5);
        pe::Real diag = r_2 / pe::Real(4.0) + h_2 / pe::Real(12.0);
        _volume = PE_PI * r_2 * height;
        _local_inertia = {
            diag, 0, 0,
            0, axis, 0,
            0, 0, diag
        };
    }

    void CylinderShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        pe::Vector3 axis = transform.getBasis().getColumn(1);
        pe::Vector3 extent = axis.getAbsolute() * (_height * pe::Real(0.5));
        extent.y += _radius * std::sqrt(axis.x * axis.x + axis.z * axis.z);
        extent.x += _radius * std::sqrt(axis.y * axis.y + axis.z * axis.z);
        extent.z += _radius * std::sqrt(axis.x * axis.x + axis.y * axis.y);
        pe::Vector3 center = transform.getOrigin();
        min = center - extent;
        max = center + extent;
    }

    bool CylinderShape::localIsInside(const pe::Vector3 &point) const {
        return point.y >= -_height * pe::Real(0.5) && point.y <= _height * pe::Real(0.5) &&
               point.x * point.x + point.z * point.z <= _radius * _radius;
    }

    void CylinderShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        pe::Vector3 local_axis = transform.getBasis().transposed() * axis;
        pe::Real half_height = _height * pe::Real(0.5);
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

}