#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    typedef pe::Array<pe::Vector3> VertexArray;

    class BoxConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b,
                                      ContactResult& result) override;
    };

} // pe_phys_collision