#pragma once

#include "physics/shape/shape.h"
#include "physics/collision/narrow_phase/contact_result.h"

namespace pe_physics_collision {

class CollisionAlgorithm {
public:
    CollisionAlgorithm() {}
    virtual ~CollisionAlgorithm() {}
    virtual bool processCollision(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
                                  pe::Transform trans_a, pe::Transform trans_b,
                                  pe::Real refScale, ContactResult& result) = 0;
};

} // pe_physics_collision

