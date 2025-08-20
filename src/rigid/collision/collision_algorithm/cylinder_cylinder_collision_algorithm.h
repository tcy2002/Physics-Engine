#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class CylinderCylinderCollisionAlgorithm : public CollisionAlgorithm {
    public:
        bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                              pe::Transform trans_a, pe::Transform trans_b,
                              pe::Real ref_scale, ContactResult& result) override;
    };

} // pe_phys_collision