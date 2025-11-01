#pragma once

#include "collision_algorithm.h"

namespace pe_physics_collision {

typedef pe::Array<pe::Vector3> VertexArray;

class BoxConvexCollisionAlgorithm : public CollisionAlgorithm {
public:
    bool processCollision(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
                          pe::Transform trans_a, pe::Transform trans_b,
                          pe::Real refScale, ContactResult& result) override;
};

} // pe_physics_collision
