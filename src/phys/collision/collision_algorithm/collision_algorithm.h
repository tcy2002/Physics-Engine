#pragma once

#include "phys/object/rigidbody.h"
#include "phys/shape/shape.h"
#include "phys/collision/narrow_phase/contact_result.h"

namespace pe_phys_collision {

    class CollisionAlgorithm {
    public:
        CollisionAlgorithm() {}
        virtual ~CollisionAlgorithm() {}
        virtual bool processCollision(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b,
                                      ContactResult& result, pe::Vector3 overlapMin, pe::Vector3 overlapMax) = 0;
    };

} // pe_phys_collision
