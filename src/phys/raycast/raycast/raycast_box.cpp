#include "raycast_box.h"
#include "phys/shape/box_shape.h"

namespace pe_phys_ray {

    bool RaycastBox::rayHitBox(const pe::Vector3& start, const pe::Vector3& direction,
                               const pe::Vector3& box_min, const pe::Vector3& box_max,
                               pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) {
        pe::Vector3 dir = pe::Vector3::max2(direction, pe::Vector3(PE_EPS, PE_EPS, PE_EPS));
        pe::Vector3 m = (box_max - start) / dir;
        pe::Vector3 n = (box_min - start) / dir;

        pe::Vector3 t_max = pe::Vector3::max2(m, n);
        pe::Vector3 t_min = pe::Vector3::min2(m, n);

        pe::Real t_enter = PE_MAX3(t_min.x, t_min.y, t_min.z);
        pe::Real t_exit = PE_MIN3(t_max.x, t_max.y, t_max.z);

        if (t_enter > t_exit || t_exit < 0) return false;

        distance = t_enter;
        hit_point = start + dir * t_enter;
        if (t_enter == t_min.x) hit_normal = -pe::Vector3::right();
        else if (t_enter == t_min.y) hit_normal = -pe::Vector3::up();
        else if (t_enter == t_min.z) hit_normal = -pe::Vector3::forward();
        else if (t_enter == t_max.x) hit_normal = pe::Vector3::right();
        else if (t_enter == t_max.y) hit_normal = pe::Vector3::up();
        else hit_normal = pe::Vector3::forward();
        return true;
    }

    bool RaycastBox::processRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real length,
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
            distance = PE_REAL_MAX;
            return false;
        }
    }

} // namespace pe_phys_ray