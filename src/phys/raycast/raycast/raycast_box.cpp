#include "raycast_box.h"
#include "phys/shape/box_shape.h"

namespace pe_phys_raycast {

    bool RaycastBox::processRaycast(const pe::Vector3& start, const pe::Vector3& direction,
                                    pe_phys_object::RigidBody* object,
                                    pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) {
        auto& trans = object->getTransform();
        auto& size = ((pe_phys_shape::BoxShape*)object->getCollisionShape())->getSize();
        pe::Vector3 start_local = trans.inverseTransform(start);
        pe::Vector3 dir_local = trans.getBasis().transposed() * direction;

        if (rayHitBox(start_local, dir_local,
                      -size / pe::Real(2.0), size / pe::Real(2.0),
                      distance, hit_point, hit_normal)) {
            hit_point = trans * hit_point;
            hit_normal = trans.getBasis() * hit_normal;
            return true;
        } else {
            hit_point = pe::Vector3::zeros();
            hit_normal = pe::Vector3::zeros();
            distance = PE_REAL_MAX;
            return false;
        }
    }

    bool RaycastBox::rayHitBox(const pe::Vector3& start, const pe::Vector3& direction,
                               const pe::Vector3& box_min, const pe::Vector3& box_max,
                               pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) {
        pe::Vector3 dir;
        dir.x = std::abs(direction.x) < PE_EPS ? PE_EPS : direction.x;
        dir.y = std::abs(direction.y) < PE_EPS ? PE_EPS : direction.y;
        dir.z = std::abs(direction.z) < PE_EPS ? PE_EPS : direction.z;

        pe::Vector3 m = (box_max - start) / dir;
        pe::Vector3 n = (box_min - start) / dir;

        pe::Vector3 t_max = pe::Vector3::max2(m, n);
        pe::Vector3 t_min = pe::Vector3::min2(m, n);

        pe::Real t_enter = PE_MAX3(t_min.x, t_min.y, t_min.z);
        pe::Real t_exit = PE_MIN3(t_max.x, t_max.y, t_max.z);

        if (t_enter > t_exit || t_exit < 0) return false;

        distance = t_enter;
        hit_point = start + dir * t_enter;
        hit_normal = pe::Vector3::zeros();
        if (t_enter == t_min.x) {
            hit_normal.x = dir.x > 0 ? pe::Real(-1.0) : pe::Real(1.0);
        } else if (t_enter == t_min.y) {
            hit_normal.y = dir.y > 0 ? pe::Real(-1.0) : pe::Real(1.0);
        } else {
            hit_normal.z = dir.z > 0 ? pe::Real(-1.0) : pe::Real(1.0);
        }

        return true;
    }

} // namespace pe_phys_ray