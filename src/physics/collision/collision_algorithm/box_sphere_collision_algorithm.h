#pragma once

#include "collision_algorithm.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/sphere_shape.h"

namespace pe_physics_collision {

class BoxSphereCollisionAlgorithm : public CollisionAlgorithm {
public:
    bool processCollision(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
                          pe::Transform trans_a, pe::Transform trans_b,
                          pe::Real refScale, ContactResult& result) override;

    static bool getClosestPoints(pe_physics_shape::SphereShape* shape_sph, pe_physics_shape::BoxShape* shape_box,
                                 const pe::Transform& trans_sph, const pe::Transform& trans_box,
                                 const pe::Vector3& center_sph, pe::Real radius_sph,
                                 pe::Real margin, ContactResult& result);
};

} // pe_physics_collision
