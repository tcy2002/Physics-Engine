#pragma once

#include "physics/collision/collision_algorithm/collision_algorithm.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/capsule_shape.h"

namespace pe_physics_collision {

class BoxCapsuleCollisionAlgorithm : public CollisionAlgorithm {
public:
    bool processCollision(pe_physics_shape::Shape *shape_a, pe_physics_shape::Shape *shape_b,
                          pe::Transform trans_a, pe::Transform trans_b,
                          pe::Real refScale, ContactResult &result) override;

    static bool getClosestPoints(pe_physics_shape::CapsuleShape* shape_a, pe_physics_shape::BoxShape* shape_b,
                                 const pe::Transform& trans_a, const pe::Transform& trans_b,
                                 pe::Real margin, ContactResult& result);
};

} // namespace pe_physics_collision
