#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class SphereSphereCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;
    };

} // pe_phys_collision