#include "raycast_cylinder.h"
#include "phys/shape/cylinder_shape.h"

namespace pe_phys_raycast {

    bool RaycastCylinder::processRaycast(const pe::Vector3& start, const pe::Vector3& direction,
                                         pe_phys_object::RigidBody* object,
                                         pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) {
        auto& trans = object->getTransform();
        auto shape = (pe_phys_shape::CylinderShape*)object->getCollisionShape();
        pe::Real radius = shape->getRadius();
        pe::Real height = shape->getHeight() / pe::Real(2.0);
        pe::Vector3 start_local = trans.inverseTransform(start);
        pe::Vector3 dir_local = trans.getBasis().transposed() * direction;

        pe::Real dist2axis;
        if (PE_APPROX_EQUAL(dir_local.x, 0) && PE_APPROX_EQUAL(dir_local.z, 0)) {
            dist2axis = std::sqrt(start_local.x * start_local.x + start_local.z * start_local.z);
            if (dist2axis > radius) {
                goto not_hit;
            } else {
                pe::Real h = dir_local.y < 0 ? height : -height;
                hit_point = start_local + (h * dir_local - dir_local * start_local) / dir_local.y;
                hit_normal = dir_local.y < 0 ? pe::Vector3::up() : -pe::Vector3::up();
                distance = (hit_point - start_local).norm();
                goto hit;
            }
        } else {
            dist2axis = std::abs(start_local.dot(dir_local.cross(pe::Vector3::up()).normalized()));
            if (dist2axis > radius) {
                goto not_hit;
            }

            pe::Real t = -(dir_local.x * start_local.x + dir_local.z * start_local.z) /
                (dir_local.x * dir_local.x + dir_local.z * dir_local.z);
            pe::Real dt = std::sqrt(radius * radius - dist2axis * dist2axis) /
                    std::sqrt(dir_local.x * dir_local.x + dir_local.z * dir_local.z);
            pe::Vector3 in_point = start_local + dir_local * (t - dt);
            pe::Vector3 out_point = start_local + dir_local * (t + dt);

            if ((in_point.y < -height && out_point.y < -height) ||
                (in_point.y > height && out_point.y > height)) {
                goto not_hit;
            } else if (in_point.y >= -height && in_point.y <= height) {
                hit_point = in_point;
                hit_normal = pe::Vector3(in_point.x, 0, in_point.z).normalized();
                distance = t - dt;
                goto hit;
            } else if (in_point.y > height) {
                hit_point = start_local + (height * dir_local - dir_local * start_local) / dir_local.y;
                hit_point.y = height;
                hit_normal = pe::Vector3::up();
                distance = (hit_point - start_local).norm();
                goto hit;
            } else {
                hit_point = start_local + (-height * dir_local - dir_local * start_local) / dir_local.y;
                hit_normal = -pe::Vector3::up();
                distance = (hit_point - start_local).norm();
                goto hit;
            }
        }

        not_hit:
        hit_point = pe::Vector3::zeros();
        hit_normal = pe::Vector3::zeros();
        distance = PE_REAL_MAX;
        return false;

        hit:
        hit_point = trans * hit_point;
        hit_normal = trans.getBasis() * hit_normal;
        return true;
    }

} // namespace pe_phys_ray
