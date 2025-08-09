#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class CompoundCompoundCollisionAlgorithm : public CollisionAlgorithm {
    public:
        bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                              pe::Transform trans_a, pe::Transform trans_b,
                              pe::Real refScale, ContactResult& result) override;

    protected:
        static bool processSubCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                        const pe::Transform& trans_a, const pe::Transform& trans_b,
                                        pe::Real refScale, ContactResult& result);
    };

} // pe_phys_collision