#pragma once

#include "raycast.h"

namespace pe_phys_raycast {

    class RaycastMesh: public Raycast {
    public:
        PE_API virtual bool processRaycast(const pe::Vector3& start, const pe::Vector3& direction,
                                           pe_phys_object::RigidBody* object, pe::Real& distance,
                                           pe::Vector3& hit_point, pe::Vector3& hit_normal) override;
        PE_API static bool rayHitTriangle(const pe::Vector3& start, const pe::Vector3& direction,
                                          const pe::Vector3& v0, const pe::Vector3& v1, const pe::Vector3& v2,
                                          pe::Real& distance, pe::Vector3& hitPoint);
    };

} // namespace pe_phys_ray