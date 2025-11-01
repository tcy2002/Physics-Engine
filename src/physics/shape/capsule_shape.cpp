#include "capsule_shape.h"

namespace pe_physics_shape {

CapsuleShape::CapsuleShape(pe::Real radius, pe::Real height): _radius(radius), _height(height) {
    const pe::Real r2 = radius * radius;
    const pe::Real h2 = height * height;
    const pe::Real r22 = r2 + r2;
    const pe::Real f1 = PE_R(2.0) * radius / (PE_R(4.0) * radius + PE_R(3.0) * height);
    const pe::Real f2 = PE_R(3.0) * height / (PE_R(4.0) * radius + PE_R(3.0) * height);
    const pe::Real sum1 = PE_R(0.4) * r22;
    const pe::Real sum2 = PE_R(0.75) * height * radius + PE_R(0.5) * h2;
    const pe::Real sum3 = PE_R(0.25) * r2 + PE_R(1.0 / 12.0) * h2;
    const pe::Real xz = f1 * (sum1 + sum2) + f2 * sum3;
    const pe::Real y = f1 * sum1 + f2 * PE_R(0.25) * r22;
    _local_inertia = {
        xz, 0, 0,
        0, y, 0,
        0, 0, xz
    };
    _volume = PE_PI * r2 * (PE_R(4.0 / 3.0) * radius + height);
}

void CapsuleShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
    auto& pos = transform.getOrigin();
    auto& rot = transform.getBasis();
    pe::Vector3 up = rot * pe::Vector3(0, _height / 2, 0);
    up = PE_ABS_VEC3(up);
    min = pos - pe::Vector3(_radius, _radius, _radius) - up;
    max = pos + pe::Vector3(_radius, _radius, _radius) + up;
}

bool CapsuleShape::localIsInside(const pe::Vector3 &point) const {
    const pe::Real half_height = _height / 2;
    const pe::Real diff_y1 = point.y - half_height;
    const pe::Real diff_y2 = point.y + half_height;
    const pe::Real x2 = point.x * point.x;
    const pe::Real z2 = point.z * point.z;
    const pe::Real r2 = _radius * _radius;
    return ((x2 + z2) < r2 && point.y < half_height && point.y > -half_height) ||
           (x2 + z2 + diff_y1 * diff_y1) < r2 ||
           (x2 + z2 + diff_y2 * diff_y2) < r2;
}

pe::Vector3 CapsuleShape::getLocalSupportPoint(const pe::Vector3 &dir) const {
    const pe::Vector3 dir_norm = dir.normalized();
    const pe::Vector3 top(0, _height / 2, 0);
    const pe::Vector3 bottom(0, -_height / 2, 0);
    return dir.y >= 0 ? top + dir_norm * _radius : bottom + dir_norm * _radius;
}

void CapsuleShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj, pe::Real &maxProj, pe::Vector3 &minPoint, pe::Vector3 &maxPoint) const {
    const pe::Matrix3& rot = transform.getBasis();
    const pe::Vector3& trans = transform.getOrigin();
    const pe::Vector3 local_axis = (rot.transposed() * axis).normalized();
    const pe::Real offset = trans.dot(axis);
    const pe::Vector3 top(0, _height / 2, 0);
    const pe::Vector3 bottom(0, -_height / 2, 0);
    const pe::Real top_proj = top.dot(local_axis);
    const pe::Real bottom_proj = bottom.dot(local_axis);
    if (top_proj > bottom_proj) {
        maxProj = offset + top_proj + _radius;
        minProj = offset + bottom_proj - _radius;
        maxPoint = transform * (top + local_axis * _radius);
        minPoint = transform * (bottom - local_axis * _radius);
    } else {
        maxProj = offset + bottom_proj + _radius;
        minProj = offset + top_proj - _radius;
        maxPoint = transform * (bottom + local_axis * _radius);
        minPoint = transform * (top - local_axis * _radius);
    }
}

} // namespace pe_physics_shape
