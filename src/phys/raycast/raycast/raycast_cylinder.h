#pragma once

#include "raycast.h"

namespace pe_phys_raycast {

    class RaycastCylinder: public Raycast {
    public:
        PE_API virtual bool processRaycast(const pe::Vector3& start, const pe::Vector3& direction,
                                           pe_phys_object::RigidBody* object, pe::Real& distance,
                                           pe::Vector3& hit_point, pe::Vector3& hit_normal) override;
    };

} // namespace pe_phys_ray