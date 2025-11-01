#pragma once

#include "raycast.h"

namespace pe_physics_raycast {

class RaycastSphere: public Raycast {
public:
    PE_API bool processRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real max_dist,
                               pe_physics_shape::Shape* shape, pe::Transform trans,
                               pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) override;
};

} // namespace pe_physics_raycast
