#pragma once

#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"

namespace pe_phys_ray {

    class Raycast {
    public:
        Raycast() {}
        virtual ~Raycast() {}
        virtual bool raycastTest(const pe::Vector3& start, const pe::Vector3& direction, pe::Real length,
                                 const pe::Array<pe_phys_object::RigidBody*>& objs,
                                 const pe::HashList<uint32_t>& excludeIds,
                                 pe::Real& distance, pe::Vector3& hitPoint, pe::Vector3& hitNormal) = 0;
    };

} // namespace pe_phys_ray