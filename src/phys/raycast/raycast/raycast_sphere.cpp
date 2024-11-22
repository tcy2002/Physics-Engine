#include "raycast_sphere.h"
#include "phys/shape/sphere_shape.h"

namespace pe_phys_raycast {

    bool RaycastSphere::processRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real max_dist,
                                       pe_phys_shape::Shape* shape, pe::Transform trans,
                                      pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) {
        const pe::Real radius = dynamic_cast<pe_phys_shape::SphereShape *>(shape)->getRadius();

        const pe::Real center2start = (trans.getOrigin() - start).norm();
        const pe::Real start2proj = (trans.getOrigin() - start).dot(direction);
        const pe::Real proj2center = std::sqrt(PE_MAX(center2start * center2start - start2proj * start2proj, 0));

        if (proj2center > radius) {
            hit_point = pe::Vector3::zeros();
            hit_normal = pe::Vector3::zeros();
            distance = PE_REAL_MAX;
            return false;
        } else {
            pe::Real proj2hit = PE_SQRT(radius * radius - proj2center * proj2center);
            distance = start2proj - proj2hit;
            if (distance <= max_dist) {
                hit_point = start + direction * distance;
                hit_normal = (hit_point - trans.getOrigin()).normalized();
                return true;
            }
            return false;
        }
    }

} // namespace pe_phys_ray