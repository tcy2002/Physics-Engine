#pragma once

#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"

namespace pe_phys_ray {

    class Raycast {
    public:
        Raycast() {}
        virtual ~Raycast() {}
        virtual bool processRaycast(const pe::Vector3& start, const pe::Vector3& direction,
                                    pe_phys_object::RigidBody* object,
                                    pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) = 0;
    };

} // namespace pe_phys_ray