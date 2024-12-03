#include "raycast_cylinder.h"
#include "phys/shape/cylinder_shape.h"

// style-checked
namespace pe_phys_raycast {

    bool RaycastCylinder::processRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real max_dist,
                                         pe_phys_shape::Shape* shape, pe::Transform trans,
                                         pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) {
        auto shape_sph = dynamic_cast<pe_phys_shape::CylinderShape *>(shape);
        const pe::Real radius = shape_sph->getRadius();
        const pe::Real height = shape_sph->getHeight() / R(2.0);
        const pe::Vector3 start_local = trans.inverseTransform(start);
        const pe::Vector3 dir_local = trans.getBasis().transpose() * direction;

        pe::Real dist2axis;
        if (PE_APPROX_EQUAL(dir_local.x(), 0) && PE_APPROX_EQUAL(dir_local.z(), 0)) {
            dist2axis = PE_SQRT(start_local.x() * start_local.x() + start_local.z() * start_local.z());
            if (dist2axis > radius) {
                goto not_hit;
            } else {
                pe::Real h = dir_local.y() < 0 ? height : -height;
                hit_point = start_local + (h * dir_local - dir_local.cwiseProduct(start_local)) / dir_local.y();
                hit_normal = pe::Vector3::UnitY();
                if (dir_local.y() > 0) hit_normal = -hit_normal;
                distance = (hit_point - start_local).norm();
                goto hit;
            }
        } else {
            dist2axis = PE_ABS(start_local.dot(dir_local.cross(pe::Vector3::UnitY()).normalized()));
            if (dist2axis > radius) {
                goto not_hit;
            }

            const pe::Real t = -(dir_local.x() * start_local.x() + dir_local.z() * start_local.z()) /
                (dir_local.x() * dir_local.x() + dir_local.z() * dir_local.z());
            const pe::Real dt = PE_SQRT(radius * radius - dist2axis * dist2axis) /
                    PE_SQRT(dir_local.x() * dir_local.x() + dir_local.z() * dir_local.z());
            const pe::Vector3 in_point = start_local + dir_local * (t - dt);
            const pe::Vector3 out_point = start_local + dir_local * (t + dt);

            if ((in_point.y() < -height && out_point.y() < -height) ||
                (in_point.y() > height && out_point.y() > height)) {
                goto not_hit;
            } else if (in_point.y() >= -height && in_point.y() <= height) {
                hit_point = in_point;
                hit_normal = pe::Vector3(in_point.x(), 0, in_point.z()).normalized();
                distance = t - dt;
                goto hit;
            } else if (in_point.y() > height) {
                hit_point = start_local + (height * dir_local - dir_local.cwiseProduct(start_local)) / dir_local.y();
                hit_point.y() = height;
                hit_normal = pe::Vector3::UnitY();
                distance = (hit_point - start_local).norm();
                goto hit;
            } else {
                hit_point = start_local + (-height * dir_local - dir_local.cwiseProduct(start_local)) / dir_local.y();
                hit_normal = -pe::Vector3::UnitY();
                distance = (hit_point - start_local).norm();
                goto hit;
            }
        }

        not_hit:
        hit_point = pe::Vector3::Zero();
        hit_normal = pe::Vector3::Zero();
        distance = PE_REAL_MAX;
        return false;

        hit:
        if (distance <= max_dist) {
            hit_point = trans * hit_point;
            hit_normal = trans.getBasis() * hit_normal;
            return true;
        }
        return false;
    }

} // namespace pe_phys_ray
