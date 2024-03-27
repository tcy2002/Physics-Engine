#pragma once

#include "raycast.h"

namespace pe_phys_ray {

    class RaycastBox: public Raycast {
    public:
        virtual bool processRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real length,
                                    pe_phys_object::RigidBody* object,
                                    pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) override;
        static bool rayHitBox(const pe::Vector3& start, const pe::Vector3& direction,
                              const pe::Vector3& box_min, const pe::Vector3& box_max,
                              pe::Real& distance, pe::Vector3& hitPoint, pe::Vector3& hitNormal);
    };

} // namespace pe_phys_ray