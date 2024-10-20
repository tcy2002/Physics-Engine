#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class CompoundCompoundCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      ContactResult& result) override;

    private:
        static CollisionAlgorithm* getCollisionAlgorithm(int index);
        static bool processSubCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                        pe::Transform& trans_a, pe::Transform& trans_b,
                                        ContactResult& result);
    };

} // pe_phys_collision