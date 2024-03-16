#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class BoxCylinderCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b,
                                      ContactResult& result) override;
    };

} // pe_phys_collision