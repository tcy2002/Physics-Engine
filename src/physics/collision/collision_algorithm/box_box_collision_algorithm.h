#pragma once

#include "collision_algorithm.h"
#include "physics/shape/shape.h"
#include "physics/collision/narrow_phase/contact_result.h"

namespace pe_physics_collision {

class BoxBoxCollisionAlgorithm : public CollisionAlgorithm {
public:
    bool processCollision(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
                          pe::Transform trans_a, pe::Transform trans_b,
                          pe::Real refScale, ContactResult& result) override;

    static bool getClosestPoints(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
                                 const pe::Transform& trans_a, const pe::Transform& trans_b,
                                 pe::Real margin, ContactResult& result);
};

} // pe_physics_collision
