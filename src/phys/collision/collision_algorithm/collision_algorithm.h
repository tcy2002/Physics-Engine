#pragma once

#include "phys/object/rigidbody.h"
#include "phys/shape/shape.h"
#include "phys/collision/narrow_phase/contact_result.h"

namespace pe_phys_collision {

    class CollisionAlgorithm {
    public:
        CollisionAlgorithm() {}
        virtual ~CollisionAlgorithm() {}
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      ContactResult& result) = 0;
    };

} // pe_phys_collision
