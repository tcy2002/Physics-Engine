#pragma once

#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"

namespace pe_phys_raycast {

    class Raycast {
    public:
        Raycast() {}
        virtual ~Raycast() {}
        virtual bool processRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real max_dist,
                                    pe_phys_shape::Shape* shape, pe::Transform trans,
                                    pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) = 0;
    };

} // namespace pe_phys_ray