#pragma once

#include "physics/physics.h"
#include "physics/object/rigidbody.h"

namespace pe_physics_raycast {

class Raycast {
public:
    Raycast() = default;
    virtual ~Raycast() = default;

    virtual bool processRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real max_dist,
                                pe_physics_shape::Shape* shape, pe::Transform trans,
                                pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) = 0;
};

} // namespace pe_physics_raycast
